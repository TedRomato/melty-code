#pragma once

#include <Arduino.h>
#include <Preferences.h>
#include <functional>

// Control logic for the handheld controller.
//
// This module provides a clean API for controlling the melty robot.
// It is intentionally decoupled from:
//  - WirelessLink (it talks through IRadio)
//  - Serial/UI formatting (Interface owns that)

class IRadio {
public:
	virtual ~IRadio() = default;
	virtual bool isConnected() const = 0;
	virtual bool isConnectionLost() const = 0;
	virtual bool send(const void* data, size_t len) = 0;
};

class Controller {
public:
	// Rotation magnitude constants (radiusMm delta)
	static constexpr float kRotateSmall = 0.050f;
	static constexpr float kRotateMedium = 0.125f;
	static constexpr float kRotateLarge = 0.250f;

	// Info received from melty (telemetry)
	struct Info {
		float revPerSec = 0.0f;      // Rotations per second (from accel)
		float updatesPerSec = 0.0f;  // Update rate on melty
		int8_t signalStrength = 0;   // RSSI or similar
	};

	// Settings stored on melty
	struct Settings {
		float radiusMm = 0.0f;       // Base radius (without rotation adjustments)
		uint8_t wifiChannel = 0;
		float motorOffsetDeg = 0.0f;
	};

	// Callbacks for async responses
	using InfoCallback = std::function<void(const Info&)>;
	using SettingsCallback = std::function<void(const Settings&)>;

	explicit Controller(IRadio& radio);

	// === Rotation Control ===
	// Stop any rotation offset (reset radius delta to zero)
	void stopRotate();
	
	// Rotate left by given magnitude (subtracts from radius)
	void rotateLeft(float magnitude);
	
	// Rotate right by given magnitude (adds to radius)  
	void rotateRight(float magnitude);

	// === Speed/Power Control ===
	// Set forward movement speed (0-100%, maps to 0-12 internally as headingMultiplier)
	// Note: Forward is always enabled, speed=0 means no forward movement
	void setForwardSpeed(float percent);
	
	// Set motor power (0-100%)
	void setMotorPower(float percent);

	// === Calibration & Adjustment ===
	// Calibrate accelerometer
	void calibrateAccel(uint16_t durationMs = 1000);
	
	// Adjust accelerometer radius (diff added to base radius mm)
	void adjustAccelRadius(float diffMm);
	
	// Set base radius directly (for loading from NVS)
	void setRadiusMm(float radiusMm);
	
	// Adjust motor forward offset (degrees added to motor lag)
	void adjustMotorFwdOffset(float degDiff);
	
	// Set motor offset directly (for loading from NVS)
	void setMotorOffsetDeg(float offsetDeg);
	
	// === NVS Settings Persistence ===
	// Save current radius and motor offset to NVS
	bool saveSettings();
	
	// Load radius and motor offset from NVS (call once at startup)
	bool loadSettings();

	// === Info & Settings Requests ===
	// Request info from melty (async - result via callback)
	void requestInfo();
	
	// Request settings from melty (async - result via callback)
	void requestSettings();

	// Set callbacks for async responses
	void setOnInfoReceive(InfoCallback callback);
	void setOnSettingsReceive(SettingsCallback callback);

	// === Fight Mode ===
	// When fight mode is active: 100% power, 0% forward, ignore other inputs
	void setFightMode(bool active);
	bool isFightMode() const;

	// === State Accessors ===
	float getRadiusMm() const;          // Base radius in mm
	float getMotorOffsetDeg() const;    // Motor lag/offset in degrees
	float getForwardSpeed() const;      // Current forward speed (0-100%)
	float getMotorPower() const;        // Current motor power (0-100%)
	float getRotationDelta() const;     // Current rotation offset
	bool isConnected() const;
	bool isConnectionLost() const;      // True if radio connection was lost

	// === Update Loop ===
	// Call from main loop to process pending sends
	void update();

	// Called from main on wireless connect
	void onConnect();

	// Called from main on RX data
	void handleRx(const uint8_t* data, size_t len);

private:
	IRadio& _radio;
	Preferences _prefs;

	// Internal state
	float _motorPower = 0.0f;           // 0-100%
	float _forwardSpeed = 0.0f;         // 0-100%, maps to headingMultiplier 0-12
	float _baseRadiusMm = 2.0f;         // Base radius
	float _rotationDelta = 0.0f;        // Current rotation offset (+ = right, - = left)
	float _motorOffsetDeg = 0.0f;       // Motor forward offset

	// Dirty flags for pending sends
	bool _controlDirty = false;
	bool _radiusDirty = false;
	bool _calibratePending = false;
	uint16_t _calibrateDurationMs = 1000;

	// Fight mode state (joystick button held)
	bool _fightMode = false;

	// Heartbeat ping to keep melty alive
	static constexpr uint32_t kPingIntervalMs = 2500;
	uint32_t _lastPingMs = 0;

	// Throttle settings/control sends
	static constexpr uint32_t kSettingsSendIntervalMs = 25;
	uint32_t _lastSettingsSendMs = 0;

	// Callbacks
	InfoCallback _onInfoReceive;
	SettingsCallback _onSettingsReceive;

	// Internal helpers
	void sendControl();
	void sendRadiusConfig();
	void sendCalibrate();
	void sendPing();
	float computeHeadingMultiplier() const;
	float computeEffectiveRadius() const;

	static float clamp(float val, float minVal, float maxVal);
};
