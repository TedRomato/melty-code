#include "Controller.h"

#include <cstring>

#include "Messages.h"

// === Constructor ===
Controller::Controller(IRadio& radio) : _radio(radio) {}

// === Rotation Control ===
void Controller::stopRotate() {
	_rotationDelta = 0.0f;
	_radiusDirty = true;
}

void Controller::rotateLeft(float magnitude) {
	_rotationDelta = -magnitude;
	_radiusDirty = true;
}

void Controller::rotateRight(float magnitude) {
	_rotationDelta = magnitude;
	_radiusDirty = true;
}

// === Speed/Power Control ===
void Controller::setForwardSpeed(float percent) {
	_forwardSpeed = clamp(percent, 0.0f, 100.0f);
	_controlDirty = true;
}

void Controller::setMotorPower(float percent) {
	_motorPower = clamp(percent, -100.0f, 100.0f);
	_controlDirty = true;
}

// === Calibration & Adjustment ===
void Controller::calibrateAccel(uint16_t durationMs) {
	_calibratePending = true;
	_calibrateDurationMs = durationMs;
}

void Controller::adjustAccelRadius(float diffMm) {
	_baseRadiusMm += diffMm;
	if (_baseRadiusMm < 0.4f) _baseRadiusMm = 0.4f;
	_radiusDirty = true;
}

void Controller::setRadiusMm(float radiusMm) {
	_baseRadiusMm = radiusMm;
	if (_baseRadiusMm < 0.4f) _baseRadiusMm = 0.4f;
	_radiusDirty = true;
}

void Controller::adjustMotorFwdOffset(float degDiff) {
	_motorOffsetDeg += degDiff;
	_motorOffsetDeg = clamp(_motorOffsetDeg, -180.0f, 180.0f);
	_controlDirty = true;
}

void Controller::setMotorOffsetDeg(float offsetDeg) {
	_motorOffsetDeg = clamp(offsetDeg, -180.0f, 180.0f);
	_controlDirty = true;
}

// === NVS Settings Persistence ===
bool Controller::saveSettings() {
	_prefs.begin("melty", false);  // false = read/write mode
	_prefs.putFloat("radius", _baseRadiusMm);
	_prefs.putFloat("offset", _motorOffsetDeg);
	_prefs.putFloat("power", _motorPower);
	_prefs.end();
	Serial.printf("Settings saved: radius=%.3fmm offset=%.1fdeg power=%.0f%%\n", 
		(double)_baseRadiusMm, (double)_motorOffsetDeg, (double)_motorPower);
	return true;
}

bool Controller::loadSettings() {
	_prefs.begin("melty", true);  // true = read-only mode
	
	// Check if settings exist
	if (!_prefs.isKey("radius")) {
		_prefs.end();
		Serial.println("No saved settings found, using defaults");
		return false;
	}
	
	_baseRadiusMm = _prefs.getFloat("radius", 2.0f);
	_motorOffsetDeg = _prefs.getFloat("offset", 0.0f);
	_motorPower = _prefs.getFloat("power", 0.0f);
	_prefs.end();
	
	// Validate
	if (_baseRadiusMm < 0.4f) _baseRadiusMm = 0.4f;
	_motorOffsetDeg = clamp(_motorOffsetDeg, -180.0f, 180.0f);
	_motorPower = clamp(_motorPower, -100.0f, 100.0f);
	
	// Mark as dirty to send to robot on connect
	_radiusDirty = true;
	_controlDirty = true;
	
	Serial.printf("Settings loaded: radius=%.3fmm offset=%.1fdeg power=%.0f%%\n", 
		(double)_baseRadiusMm, (double)_motorOffsetDeg, (double)_motorPower);
	return true;
}

// === Info & Settings Requests ===
void Controller::requestInfo() {
	if (!_radio.isConnected()) return;
	
	InfoRequestMsg msg{};
	msg.kind = static_cast<uint8_t>(MsgKind::INFO_REQUEST);
	_radio.send(&msg, sizeof(msg));
}

void Controller::requestSettings() {
	if (!_radio.isConnected()) return;
	
	SettingsRequestMsg msg{};
	msg.kind = static_cast<uint8_t>(MsgKind::SETTINGS_REQUEST);
	_radio.send(&msg, sizeof(msg));
}

void Controller::setOnInfoReceive(InfoCallback callback) {
	_onInfoReceive = callback;
}

void Controller::setOnSettingsReceive(SettingsCallback callback) {
	_onSettingsReceive = callback;
}

// === State Accessors ===
float Controller::getRadiusMm() const {
	return _baseRadiusMm;
}

float Controller::getMotorOffsetDeg() const {
	return _motorOffsetDeg;
}

float Controller::getForwardSpeed() const {
	return _forwardSpeed;
}

float Controller::getMotorPower() const {
	return _motorPower;
}

float Controller::getRotationDelta() const {
	return _rotationDelta;
}

bool Controller::isConnected() const {
	return _radio.isConnected();
}

bool Controller::isConnectionLost() const {
	return _radio.isConnectionLost();
}

// === Fight Mode ===
void Controller::setFightMode(bool active) {
	_fightMode = active;
	_controlDirty = true;
	Serial.printf("Fight mode: %s\n", active ? "ACTIVE" : "OFF");
}

bool Controller::isFightMode() const {
	return _fightMode;
}

// === Update Loop ===
void Controller::update() {
	const uint32_t nowMs = millis();
	
	// Process calibration request first
	if (_calibratePending) {
		sendCalibrate();
		_calibratePending = false;
	}
	
	// Send control/radius if dirty (throttled to avoid radio spam)
	if (_lastSettingsSendMs == 0 || (nowMs - _lastSettingsSendMs) >= kSettingsSendIntervalMs) {
		bool sent = false;
		if (_controlDirty) {
			sendControl();
			_controlDirty = false;
			sent = true;
		}
		if (_radiusDirty) {
			sendRadiusConfig();
			_radiusDirty = false;
			sent = true;
		}
		if (sent) {
			_lastSettingsSendMs = nowMs;
		}
	}
	
	// Send periodic ping to keep melty alive (dead man's switch heartbeat)
	if (_radio.isConnected() && (nowMs - _lastPingMs >= kPingIntervalMs)) {
		sendPing();
		_lastPingMs = nowMs;
	}
}

void Controller::onConnect() {
	// Send current state on connect
	_controlDirty = true;
	_radiusDirty = true;
}

void Controller::handleRx(const uint8_t* data, size_t len) {
	if (!data || len < 1) return;

	const uint8_t kind = data[0];
	
	if (kind == static_cast<uint8_t>(MsgKind::ACCEL_OMEGA)) {
		if (len < sizeof(AccelOmegaMsg)) return;
		
		AccelOmegaMsg msg{};
		std::memcpy(&msg, data, sizeof(msg));
		
		// Convert to Info and call callback
		if (_onInfoReceive) {
			Info info{};
			info.revPerSec = msg.revPerSec;
			// updatesPerSec and signalStrength would come from INFO_RESPONSE
			_onInfoReceive(info);
		}
		return;
	}
	
	if (kind == static_cast<uint8_t>(MsgKind::INFO_RESPONSE)) {
		if (len < sizeof(InfoResponseMsg)) return;
		
		InfoResponseMsg msg{};
		std::memcpy(&msg, data, sizeof(msg));
		
		if (_onInfoReceive) {
			Info info{};
			info.revPerSec = msg.revPerSec;
			info.updatesPerSec = msg.updatesPerSec;
			info.signalStrength = msg.signalStrength;
			_onInfoReceive(info);
		}
		return;
	}
	
	if (kind == static_cast<uint8_t>(MsgKind::SETTINGS_RESPONSE)) {
		if (len < sizeof(SettingsResponseMsg)) return;
		
		SettingsResponseMsg msg{};
		std::memcpy(&msg, data, sizeof(msg));
		
		if (_onSettingsReceive) {
			Settings settings{};
			settings.radiusMm = msg.radiusMm;
			settings.wifiChannel = msg.wifiChannel;
			settings.motorOffsetDeg = msg.motorOffsetDeg;
			_onSettingsReceive(settings);
		}
		return;
	}
}

// === Private Helpers ===
void Controller::sendControl() {
	if (!_radio.isConnected()) return;

	// In fight mode: 100% power, 0% forward (headingMultiplier = 0)
	float power = _fightMode ? ((_motorPower < 0.0f) ? -50.0f : 50.0f) : _motorPower;
	float headingMult = _fightMode ? 0.0f : computeHeadingMultiplier();

	ControlMsg msg{};
	msg.kind = static_cast<uint8_t>(MsgKind::CONTROL);
	msg._reserved1 = 1; // Reserved, previously forwardEnabled
	msg.leftPercent = power;
	msg.rightPercent = power / (_fightMode ? 2.0f : 1.0f); // Right motor gets half power to increase radius in fight mode
	msg.headingMultiplier = headingMult;
	msg.motorLagDeg = _motorOffsetDeg;
	_radio.send(&msg, sizeof(msg));
}

void Controller::sendRadiusConfig() {
	if (!_radio.isConnected()) return;

	RadiusConfigMsg msg{};
	msg.kind = static_cast<uint8_t>(MsgKind::RADIUS_CONFIG);
	msg.radiusM = computeEffectiveRadius() * 1e-3f;
	_radio.send(&msg, sizeof(msg));
}

void Controller::sendCalibrate() {
	if (!_radio.isConnected()) return;

	AccelCalibrateMsg msg{};
	msg.kind = static_cast<uint8_t>(MsgKind::ACCEL_CALIBRATE);
	msg.durationMs = _calibrateDurationMs;
	_radio.send(&msg, sizeof(msg));																																																
}

void Controller::sendPing() {
	if (!_radio.isConnected()) return;

	PingMsg msg{};
	msg.kind = static_cast<uint8_t>(MsgKind::PING);
	_radio.send(&msg, sizeof(msg));
}

float Controller::computeHeadingMultiplier() const {
	// Map forward speed 0-100% to headingMultiplier 0-12
	return (_forwardSpeed / 100.0f) * 6.0f;
}

float Controller::computeEffectiveRadius() const {
	return _baseRadiusMm + _rotationDelta;
}

float Controller::clamp(float val, float minVal, float maxVal) {
	if (val < minVal) return minVal;
	if (val > maxVal) return maxVal;
	return val;
}

