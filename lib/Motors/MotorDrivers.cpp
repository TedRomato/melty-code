#include "MotorDrivers.h"

#include <Arduino.h>

namespace {
constexpr int kArmPulseUs = 1000;

static float clampPercent(float percent) {
	if (percent < 0.0f) return 0.0f;
	if (percent > 100.0f) return 100.0f;
	return percent;
}
}

MotorDrivers::MotorDrivers(int leftPin, int rightPin)
		: _leftPin(leftPin),
			_rightPin(rightPin),
			_minUs(1000),
			_maxUs(2000),
			_accelRate(0.0f),
			_leftCurrent(0.0f),
			_rightCurrent(0.0f),
			_leftTarget(0.0f),
			_rightTarget(0.0f),
			_leftHoldUntil(0),
			_rightHoldUntil(0),
			_lastUpdateMs(0) {}

void MotorDrivers::init(int minUs, int maxUs, int armMs, float accelPercentPerSec) {
	_minUs = minUs;
	_maxUs = maxUs;
	_accelRate = accelPercentPerSec;

	// Use the same pulse bounds for attach so that writeMicroseconds() stays in-range.
	_leftEsc.attach(_leftPin, _minUs, _maxUs);
	_rightEsc.attach(_rightPin, _minUs, _maxUs);

	// Arming: most ESCs expect a low-throttle pulse for some time.
	_leftEsc.writeMicroseconds(kArmPulseUs);
	_rightEsc.writeMicroseconds(kArmPulseUs);
	if (armMs > 0) {
		delay(static_cast<unsigned long>(armMs));
	}

	_leftCurrent = 0.0f;
	_rightCurrent = 0.0f;
	_leftTarget = 0.0f;
	_rightTarget = 0.0f;
	_leftHoldUntil = 0;
	_rightHoldUntil = 0;

	// Ensure we start at "0%" = minUs.
	_leftEsc.writeMicroseconds(percentToUs(0.0f));
	_rightEsc.writeMicroseconds(percentToUs(0.0f));

	_lastUpdateMs = millis();
}

void MotorDrivers::update() {
	const unsigned long nowMs = millis();
	if (_lastUpdateMs == 0) {
		_lastUpdateMs = nowMs;
	}

	const unsigned long elapsedMs = nowMs - _lastUpdateMs;
	_lastUpdateMs = nowMs;

	const float dt = static_cast<float>(elapsedMs) / 1000.0f;

	// Handle hold timers expiring.
	if (_leftHoldUntil != 0 && static_cast<long>(nowMs - _leftHoldUntil) >= 0) {
		_leftHoldUntil = 0;
		_leftTarget = 0.0f;
	}
	if (_rightHoldUntil != 0 && static_cast<long>(nowMs - _rightHoldUntil) >= 0) {
		_rightHoldUntil = 0;
		_rightTarget = 0.0f;
	}

	stepMotor(_leftCurrent, _leftTarget, nowMs, dt);
	stepMotor(_rightCurrent, _rightTarget, nowMs, dt);

	// Serial.printf("Motor update: left=%d us right=%d us (left=%.2f%% right=%.2f%%)\n", percentToUs(_leftCurrent), percentToUs(_rightCurrent), _leftCurrent, _rightCurrent);
	_leftEsc.writeMicroseconds(percentToUs(_leftCurrent));
	_rightEsc.writeMicroseconds(percentToUs(_rightCurrent));
}

void MotorDrivers::setSpeed(float percent, unsigned long holdMs, bool acceleration) {
	setLeftSpeed(percent, holdMs, acceleration);
	setRightSpeed(percent, holdMs, acceleration);
}

float MotorDrivers::getLeftSpeed() const {
	return _leftCurrent;
}

float MotorDrivers::getRightSpeed() const {
	return _rightCurrent;
}

bool MotorDrivers::areMotorsOff() const {
	return (_leftCurrent == 0.0f) && (_rightCurrent == 0.0f);
}

void MotorDrivers::setLeftSpeed(float percent, unsigned long holdMs, bool acceleration) {
	_leftTarget = clampPercent(percent);
	if (holdMs > 0) {
		_leftHoldUntil = millis() + holdMs;
	} else {
		_leftHoldUntil = 0;
	}

	if (!acceleration) {
		_leftCurrent = _leftTarget;
		_leftEsc.writeMicroseconds(percentToUs(_leftCurrent));
	}
}

void MotorDrivers::setRightSpeed(float percent, unsigned long holdMs, bool acceleration) {
	_rightTarget = clampPercent(percent);
	if (holdMs > 0) {
		_rightHoldUntil = millis() + holdMs;
	} else {
		_rightHoldUntil = 0;
	}

	if (!acceleration) {
		_rightCurrent = _rightTarget;
		_rightEsc.writeMicroseconds(percentToUs(_rightCurrent));
	}
}

void MotorDrivers::sweepSpeed(int fromUs, int toUs, unsigned long durationMs) {
	// Simple blocking sweep helper (useful for ESC testing).
	Serial.printf("MotorDrivers::sweepSpeed(fromUs=%d, toUs=%d, durationMs=%lu)\n", fromUs, toUs, durationMs);
	if (durationMs == 0) {
		_leftEsc.writeMicroseconds(toUs);
		_rightEsc.writeMicroseconds(toUs);
		Serial.printf("  us=%d\n", toUs);
		return;
	}

	const unsigned long start = millis();
	unsigned long lastPrint = 0;
	while (static_cast<unsigned long>(millis() - start) < durationMs) {
		const unsigned long t = millis() - start;
		const float alpha = static_cast<float>(t) / static_cast<float>(durationMs);
		const int us = static_cast<int>(fromUs + (toUs - fromUs) * alpha);
		_leftEsc.writeMicroseconds(us);
		_rightEsc.writeMicroseconds(us);
		if (lastPrint == 0 || (t - lastPrint) >= 50) {
			Serial.printf("  t=%lums us=%d\n", t, us);
			lastPrint = t;
		}
		delay(5);
	}

	_leftEsc.writeMicroseconds(toUs);
	_rightEsc.writeMicroseconds(toUs);
	Serial.printf("  t=%lums us=%d (final)\n", durationMs, toUs);
}

int MotorDrivers::percentToUs(float percent) const {
	const float p = clampPercent(percent);
	const float span = static_cast<float>(_maxUs - _minUs);
	const float usF = static_cast<float>(_minUs) + (span * (p / 100.0f));

	int us = static_cast<int>(usF + 0.5f);
	if (us < _minUs) us = _minUs;
	if (us > _maxUs) us = _maxUs;
	return us;
}

void MotorDrivers::stepMotor(float &current, float &target, unsigned long /*nowMs*/, float dt) {
	target = clampPercent(target);
	current = clampPercent(current);

	if (dt <= 0.0f) {
		return;
	}

	// If no accel limit is configured, snap to target.
	if (!isfinite(_accelRate) || _accelRate <= 0.0f) {
		current = target;
		return;
	}

	const float maxStep = _accelRate * dt;
	const float delta = target - current;
	if (fabsf(delta) <= maxStep) {
		current = target;
	} else {
		current += (delta > 0.0f) ? maxStep : -maxStep;
	}

	current = clampPercent(current);
}



/*
// #include <Arduino.h>
// #include <MotorDrivers.h>

// MotorDrivers motors(3, 4); // Example pins
// void setup() {
//   Serial.begin(115200);
//   delay(500);
//   Serial.println("\n\n=== MotorDrivers Test ===");
//   Serial.println("\nArming...");
//   motors.init(1000, 1700, 10000, 50.0f); // minUs, maxUs, armMs, accelPercentPerSec  
//   Serial.println("\nStarting...");
//   // motors.sweepSpeed(1000, 2000, 30000); // Sweep from 1000us to 2000us over 3 seconds // 1050 - 1700

//   motors.setSpeed(50.0f, 5000); // Set speed to 50% for 5 seconds
  
// }
// int i = 0;
// void loop() {
//   motors.update();
//   if( motors.areMotorsOff() ) {
//     motors.setLeftSpeed(75.0f); // Set speed to 75% indefinitely
//     motors.setRightSpeed(25.0f); // Set speed to 25% indefinitely
//   } 
//   // put your main code here, to run repeatedly:
// }


*/