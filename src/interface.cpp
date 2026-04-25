#include "Interface.h"

// OLED display dimensions (72x40 with offsets 33,12)
static constexpr int kScreenWidth = 72;
static constexpr int kScreenHeight = 40;

Interface::Interface(Controller& controller, const Config& config)
	: _controller(controller), _config(config) {}

void Interface::begin() {
	Serial.println("\n=== Unified Interface ===");
	printHelp();
	
	// Always initialize I2C and OLED display
	Serial.println("\n--- Display Init ---");
	Serial.printf("I2C: SDA=%d SCL=%d OLED_ADDR=0x%02X\n", 
		_config.i2cSda, _config.i2cScl, _config.oledAddr);
	Wire.begin(_config.i2cSda, _config.i2cScl);
	initDisplay();
	
	if (_config.enableHardware) {
		Serial.println("\n--- Hardware Init ---");
		Serial.printf("Joystick: X=%d Y=%d BTN=%d\n",
			_config.joystickXPin, _config.joystickYPin, _config.joystickBtnPin);
		Serial.printf("Encoder1 (Power): A=%d B=%d\n", _config.enc1PinA, _config.enc1PinB);
		Serial.printf("Encoder2 (Radius): A=%d B=%d\n", _config.enc2PinA, _config.enc2PinB);
		Serial.printf("Encoder3 (Offset): A=%d B=%d\n", _config.enc3PinA, _config.enc3PinB);
		Serial.printf("Save button: %d\n", _config.saveBtnPin);
		Serial.printf("Buzzer pin: %d\n", _config.buzzerPin);
		
		// Initialize joystick pins
		pinMode(_config.joystickXPin, INPUT);
		pinMode(_config.joystickYPin, INPUT);
		if (_config.joystickBtnPin >= 0) {
			pinMode(_config.joystickBtnPin, INPUT_PULLUP);
		}
		
		// Initialize save button
		if (_config.saveBtnPin >= 0) {
			pinMode(_config.saveBtnPin, INPUT_PULLUP);
		}
		
		// Initialize buzzer
		if (_config.buzzerPin >= 0) {
			pinMode(_config.buzzerPin, OUTPUT);
			digitalWrite(_config.buzzerPin, LOW);
		}
		
		// Initialize encoder pins with pullups
		pinMode(_config.enc1PinA, INPUT_PULLUP);
		pinMode(_config.enc1PinB, INPUT_PULLUP);
		pinMode(_config.enc2PinA, INPUT_PULLUP);
		pinMode(_config.enc2PinB, INPUT_PULLUP);
		pinMode(_config.enc3PinA, INPUT_PULLUP);
		pinMode(_config.enc3PinB, INPUT_PULLUP);
		
		// Read initial encoder states
		_enc1LastState = (digitalRead(_config.enc1PinA) << 1) | digitalRead(_config.enc1PinB);
		_enc2LastState = (digitalRead(_config.enc2PinA) << 1) | digitalRead(_config.enc2PinB);
		_enc3LastState = (digitalRead(_config.enc3PinA) << 1) | digitalRead(_config.enc3PinB);
		
		Serial.println("Hardware initialized");
	} else {
		Serial.println("Hardware inputs disabled - keyboard only mode");
	}
	
	// Set up controller callbacks
	_controller.setOnInfoReceive([this](const Controller::Info& info) {
		onInfoReceived(info);
	});
	
	_controller.setOnSettingsReceive([this](const Controller::Settings& settings) {
		onSettingsReceived(settings);
	});
	
	// Initial display refresh
	if (_displayInitialized) {
		refreshDisplay();
	}
}

void Interface::update() {
	const uint32_t nowMs = millis();
	
	// Read hardware inputs if enabled
	if (_config.enableHardware) {
		// Always read button (to detect fight mode toggle)
		readButton();
		
		// Skip other inputs when in fight mode
		if (!_controller.isFightMode()) {
			readJoystick();
			readEncoders();
			// Also allow keyboard input in travel mode
			readKeyboard();
			checkRotateRelease();
		}
		
		readSaveButton();
		updateBuzzer(nowMs);
	} else {
		// Keyboard-only mode: check fight mode before processing
		if (!_controller.isFightMode()) {
			readKeyboard();
			checkRotateRelease();
		}
	}
	
	// Request settings periodically (info is pushed by melty)
	if (nowMs - _lastInfoRequestMs >= kInfoRequestMs) {
		_lastInfoRequestMs = nowMs;
		if (_controller.isConnected()) {
			_controller.requestSettings();
		}
	}
	
	// Refresh display periodically
	if (_displayInitialized) {
		if (nowMs - _lastDisplayRefreshMs >= kDisplayRefreshMs) {
			_lastDisplayRefreshMs = nowMs;
			refreshDisplay();
		}
	}
}

// ==================== DISPLAY METHODS ====================

void Interface::initDisplay() {
	Serial.println("Initializing OLED display...");
	_display = new U8G2_SSD1306_72X40_ER_F_HW_I2C(U8G2_R2, /* reset=*/ U8X8_PIN_NONE);
	
	Serial.println("Calling display->begin()...");
	if (!_display->begin()) {
		Serial.println("ERROR: U8g2 display initialization failed!");
		Serial.println("Check I2C wiring and address.");
		_displayInitialized = false;
		return;
	}
	Serial.println("OLED initialized successfully!");
	_displayInitialized = true;

	
	// Clear entire display including any noise
	_display->clearBuffer();
	_display->setDrawColor(0);  // Black
	_display->drawBox(0, 0, 128, 64);  // Clear full framebuffer
	_display->setDrawColor(1);  // White
	_display->setFont(u8g2_font_5x7_tf);
	_display->drawStr(3, 15, "Melty Ctrl");
	_display->drawStr(3, 25, "Init...");
	_display->sendBuffer();
}

void Interface::refreshDisplay() {
	if (!_display || !_displayInitialized) return;
	
	const uint32_t nowMs = millis();
	
	// Check if calibration finished
	if (_isCalibrating && (nowMs - _calibrateStartMs >= kCalibrateDurationMs)) {
		_isCalibrating = false;
	}
	
	// Print status to Serial for debugging (less frequently)
	static uint32_t lastSerialPrintMs = 0;
	if (nowMs - lastSerialPrintMs >= 2000) {
		lastSerialPrintMs = nowMs;
		Serial.println("\n--- Status ---");
		Serial.printf("Connected: %s\n", _controller.isConnected() ? "YES" : "NO");
		Serial.printf("Motor power: %.0f%%\n", (double)_controller.getMotorPower());
		Serial.printf("Forward speed: %.0f%%\n", (double)_controller.getForwardSpeed());
		Serial.printf("Base radius: %.3f mm\n", (double)_controller.getRadiusMm());
		Serial.printf("Rotation delta: %.3f mm\n", (double)_controller.getRotationDelta());
		Serial.printf("Motor offset: %.1f deg\n", (double)_controller.getMotorOffsetDeg());
		if (_hasInfo) {
			Serial.printf("Rev/s: %.3f, Updates/s: %.0f\n", 
				(double)_lastInfo.revPerSec, (double)_lastInfo.updatesPerSec);
		}
		Serial.println("--------------");
	}
	
	drawStatusScreen();
}

void Interface::drawStatusScreen() {
	// Clear entire buffer including noise on left
	_display->clearBuffer();
	_display->setDrawColor(0);  // Black
	_display->drawBox(0, 0, 128, 64);  // Clear full framebuffer
	_display->setDrawColor(1);  // White
	_display->setFont(u8g2_font_5x7_tf);
	
	char buf[20];  // 72px / 5px per char ≈ 14 chars max
	
	// Show calibrating screen if calibrating
	if (_isCalibrating) {
		_display->drawStr(3, 8, "Calibrating");
		_display->drawStr(3, 16, "...");
		_display->sendBuffer();
		return;
	}
	
	// ===== Row 1 (y=8): Connection / RPS / Rotation / Fwd =====
	if (!_controller.isConnected() || _controller.isConnectionLost()) {
		_display->drawStr(3, 8, "DISCONNECTED");
	} else {
		// RPS
		if (_hasInfo) {
			snprintf(buf, sizeof(buf), "%.0f", (double)_lastInfo.revPerSec);
		} else {
			snprintf(buf, sizeof(buf), "--");
		}
		_display->drawStr(3, 8, buf);
		
		// Rotation indicator: <<</<</</-/>/>>/>>>
		float rotDelta = _controller.getRotationDelta();
		const char* rotStr = "-";
		if (rotDelta <= -Controller::kRotateLarge) {
			rotStr = "<<<";
		} else if (rotDelta <= -Controller::kRotateMedium) {
			rotStr = "<<";
		} else if (rotDelta <= -Controller::kRotateSmall) {
			rotStr = "<";
		} else if (rotDelta >= Controller::kRotateLarge) {
			rotStr = ">>>";
		} else if (rotDelta >= Controller::kRotateMedium) {
			rotStr = ">>";
		} else if (rotDelta >= Controller::kRotateSmall) {
			rotStr = ">";
		}
		_display->drawStr(25, 8, rotStr);
		
		// Forward %
		snprintf(buf, sizeof(buf), "%3.0f%%", (double)_controller.getForwardSpeed());
		_display->drawStr(50, 8, buf);
	}
	
	// ===== Row 2 (y=16): Power | Lag =====
	snprintf(buf, sizeof(buf), "P:%3.0f%%", (double)_controller.getMotorPower());
	_display->drawStr(3, 16, buf);
	
	snprintf(buf, sizeof(buf), "L:%+.0f", (double)_controller.getMotorOffsetDeg());
	_display->drawStr(3, 24, buf);
	
	// ===== Row 3 (y=24): Radius | dRadius =====
	float radius = _controller.getRadiusMm();
	snprintf(buf, sizeof(buf), "R:%.3f", (double)radius);	
	_display->drawStr(36, 16, buf);
	
	float rotDelta = _controller.getRotationDelta();
	snprintf(buf, sizeof(buf), "d:%+.3f", (double)rotDelta);

	/* Remove leading zero: d:+0.15 → d:+.15, d:-0.15 → d:-.15 */
	char *p = strchr(buf, ':');
	if (p && (p[1] == '+' || p[1] == '-') && p[2] == '0' && p[3] == '.') {
		memmove(p + 2, p + 3, strlen(p + 3) + 1);
	}

	_display->drawStr(36, 24, buf);
	
	_display->sendBuffer();
}

// ==================== HARDWARE INPUT METHODS ====================

void Interface::readJoystick() {
	int xValue = analogRead(_config.joystickXPin);
	int yValue = analogRead(_config.joystickYPin);
	// Serial.printf("Joystick: X=%d Y=%d\n", xValue, yValue);
	applyJoystickRotation(xValue);
	applyJoystickForward(yValue);
}

void Interface::applyJoystickRotation(int xValue) {
	int delta = xValue - _joystickCenterX;

	// Calculate max range from calibrated center (use smaller of left/right range)
	int maxRangeRight = 4095 - _joystickCenterX;  // Max positive delta
	int maxRangeLeft = _joystickCenterX;           // Max negative delta
	int maxRange = (delta >= 0) ? maxRangeRight : maxRangeLeft;
	
	// Calculate dynamic thresholds based on calibrated range
	int smallThreshold = static_cast<int>(maxRange * kJoystickSmallThresholdPct);
	int mediumThreshold = static_cast<int>(maxRange * kJoystickMediumThresholdPct);

	// Serial.printf("Joystick: X=%d Deadzone=%d Delta=%d SmallThreshold=%d MediumThreshold=%d\n", xValue, kJoystickDeadzone, abs(delta), smallThreshold, mediumThreshold);
	
	if (abs(delta) < kJoystickDeadzone) {
		if (_controller.getRotationDelta() != 0.0f) {
			_controller.stopRotate();
		}
		return;
	}
	
	float magnitude;
	int absDelta = abs(delta);
	if (absDelta < smallThreshold) {
		magnitude = Controller::kRotateSmall;
	} else if (absDelta < mediumThreshold) {
		magnitude = Controller::kRotateMedium;
	} else {
		magnitude = Controller::kRotateLarge;
	}
	
	if (delta > 0) {
		_controller.rotateRight(magnitude);
	} else {
		_controller.rotateLeft(magnitude);
	}
}

void Interface::applyJoystickForward(int yValue) {
	int delta = yValue - _joystickCenterY;
	

	// Serial.printf("Joystick: Y=%d Deadzone=%d Delta=%d\n", yValue, kJoystickDeadzone, delta);
	if (delta < kJoystickDeadzone) {
		_controller.setForwardSpeed(0.0f);
		return;
	}
	
	// Map joystick position directly to forward speed percentage
	// Subtract deadzone from delta for effective range
	int effectiveDelta = delta - kJoystickDeadzone;
	
	// Max range from calibrated center to max ADC value, minus deadzone
	const int maxRange = (4095 - _joystickCenterY) - kJoystickDeadzone;
	
	// Map to 0-100% speed
	float speedPercent = ((float)effectiveDelta / (float)maxRange) * 100.0f;
	
	// Clamp to valid range
	if (speedPercent > 100.0f) speedPercent = 100.0f;
	if (speedPercent < 0.0f) speedPercent = 0.0f;
	
	_controller.setForwardSpeed(speedPercent);
	// Serial.printf("Speed=%.2f\n", speedPercent);
}

void Interface::pollEncoder(int8_t pinA, int8_t pinB, uint8_t& lastState, volatile int32_t& count) {
	uint8_t newState = (digitalRead(pinA) << 1) | digitalRead(pinB);
	
	if (newState != lastState) {
		static const int8_t encTable[16] = {
			 0, -1,  1,  0,
			 1,  0,  0, -1,
			-1,  0,  0,  1,
			 0,  1, -1,  0
		};
		
		uint8_t idx = (lastState << 2) | newState;
		count += encTable[idx];
		lastState = newState;
	}

}

// Helper to calculate accelerated steps based on rotation speed
// Returns the effective number of steps after applying acceleration logic
int32_t Interface::calculateAcceleratedSteps(int32_t rawSteps, uint32_t nowMs,
		uint32_t& lastEventMs, int& fastCount, bool& accelActive, int& accelToggle) {
	
	// Check if this rotation was fast (within timeout of last one)
	uint32_t timeSinceLast = nowMs - lastEventMs;
	lastEventMs = nowMs;
	
	if (timeSinceLast < kEncoderAccelTimeoutMs) {
		// Fast rotation - increment counter
		fastCount++;
		if (fastCount >= kEncoderAccelThreshold && !accelActive) {
			accelActive = true;
			accelToggle = 0;
			Serial.println("Encoder: Acceleration activated!");
		}
	} else {
		// Slow rotation - reset acceleration
		if (accelActive) {
			Serial.println("Encoder: Acceleration deactivated (timeout)");
		}
		fastCount = 0;
		accelActive = false;
		accelToggle = 0;
	}
	
	// Apply acceleration if active
	if (accelActive) {
		accelToggle++;
		return rawSteps * kEncoderAccelMultiplier;
	}
	
	return rawSteps;
}

void Interface::readEncoders() {
	const uint32_t nowMs = millis();
	
	pollEncoder(_config.enc1PinA, _config.enc1PinB, _enc1LastState, _enc1Count);
	pollEncoder(_config.enc2PinA, _config.enc2PinB, _enc2LastState, _enc2Count);
	pollEncoder(_config.enc3PinA, _config.enc3PinB, _enc3LastState, _enc3Count);

	// Serial.printf("Encoders: %ld, %ld, %ld\n", (long)_enc1Count, (long)_enc2Count, (long)_enc3Count);

	// Encoder 1: Motor Power (1% per detent)
	int32_t enc1Delta = _enc1Count - _lastEnc1Count;
	if (enc1Delta >= 4 || enc1Delta <= -4) {
		_lastEnc1Count = _enc1Count;
		int32_t rawSteps = enc1Delta / 4;
		if (rawSteps != 0) {
			int32_t steps = calculateAcceleratedSteps(rawSteps, nowMs,
				_enc1LastEventMs, _enc1FastCount, _enc1AccelActive, _enc1AccelToggle);
			float newPower = _controller.getMotorPower() + (float)steps;
			_controller.setMotorPower(newPower);
			Serial.printf("Encoder1: Motor power -> %.0f%% (steps=%ld)\n", (double)_controller.getMotorPower(), (long)steps);
		}
	}
	
	// Encoder 2: Radius (0.005mm per detent)
	int32_t enc2Delta = _enc2Count - _lastEnc2Count;
	if (enc2Delta >= 4 || enc2Delta <= -4) {
		_lastEnc2Count = _enc2Count;
		int32_t rawSteps = enc2Delta / 4;
		if (rawSteps != 0) {
			int32_t steps = calculateAcceleratedSteps(rawSteps, nowMs,
				_enc2LastEventMs, _enc2FastCount, _enc2AccelActive, _enc2AccelToggle);
			_controller.adjustAccelRadius((float)steps * 0.005f);
			Serial.printf("Encoder2: Radius -> %.3f mm (steps=%ld)\n", (double)_controller.getRadiusMm(), (long)steps);
		}
	}
	
	// Encoder 3: Motor Offset (1 degree per detent)
	int32_t enc3Delta = _enc3Count - _lastEnc3Count;
	if (enc3Delta >= 4 || enc3Delta <= -4) {
		_lastEnc3Count = _enc3Count;
		int32_t rawSteps = enc3Delta / 4;
		if (rawSteps != 0) {
			int32_t steps = calculateAcceleratedSteps(rawSteps, nowMs,
				_enc3LastEventMs, _enc3FastCount, _enc3AccelActive, _enc3AccelToggle);
			_controller.adjustMotorFwdOffset((float)steps);
			Serial.printf("Encoder3: Motor offset -> %.1f deg (steps=%ld)\n", (double)_controller.getMotorOffsetDeg(), (long)steps);
		}
	}
}

void Interface::beep(uint32_t durationMs) {
	if (_config.buzzerPin < 0) return;
	const uint32_t nowMs = millis();
	_buzzerOn = true;
	_buzzerOffAtMs = nowMs + durationMs;
	digitalWrite(_config.buzzerPin, HIGH);
}

void Interface::updateBuzzer(uint32_t nowMs) {
	if (!_buzzerOn) return;
	if (static_cast<int32_t>(nowMs - _buzzerOffAtMs) >= 0) {
		_buzzerOn = false;
		digitalWrite(_config.buzzerPin, LOW);
	}
}

void Interface::readButton() {
	if (_config.joystickBtnPin < 0) return;
	
	// Button is HIGH when pressed (fight mode trigger)
	bool btnPressed = (digitalRead(_config.joystickBtnPin) == LOW);
	if (btnPressed ) {
		_btnPressedMs = millis();
		Serial.println("Button: FIGHT MODE ACTIVE");
		_controller.setFightMode(true);
	} else if (!btnPressed && _lastBtnState) {
		uint32_t pressDuration = millis() - _btnPressedMs;
		Serial.printf("Button: FIGHT MODE OFF (held %lu ms)\n", (unsigned long)pressDuration);
		_controller.setFightMode(false);
	}
	
	_lastBtnState = btnPressed;
}

void Interface::readSaveButton() {
	if (_config.saveBtnPin < 0) return;
	
	bool btnPressed = (digitalRead(_config.saveBtnPin) == LOW);
	uint32_t nowMs = millis();
	
	// Button just pressed - start timing and reset calibration accumulators
	if (btnPressed && !_lastSaveBtnState) {
		_saveBtnPressedMs = nowMs;
		_joystickCalibSumX = 0;
		_joystickCalibSumY = 0;
		_joystickCalibCount = 0;
		Serial.println("Save button: PRESSED");
	}
	
	// While button is held, accumulate joystick values for center calibration
	uint32_t heldTime = nowMs - _saveBtnPressedMs;
	if (btnPressed) {
		_joystickCalibSumX += analogRead(_config.joystickXPin);
		_joystickCalibSumY += analogRead(_config.joystickYPin);
		_joystickCalibCount++;

		// Long press threshold reached - beep once while still holding
		if (nowMs - _saveBtnPressedMs >= kSaveBtnLongPressMs) {
			beep(kBeepLongMs);
			// Long press: calibrate accelerometer + joystick center
			if (_joystickCalibCount > 0) {
				_joystickCenterX = _joystickCalibSumX / _joystickCalibCount;
				_joystickCenterY = _joystickCalibSumY / _joystickCalibCount;
				Serial.printf("Joystick center calibrated: X=%d Y=%d (from %ld samples)\n",
					_joystickCenterX, _joystickCenterY, (long)_joystickCalibCount);
			}
			if (_controller.isConnected()) {
				_controller.calibrateAccel();
				_isCalibrating = true;
				_calibrateStartMs = nowMs;
				Serial.println("Accelerometer calibration triggered (long press)");
			} else {
				Serial.println("Not connected - skipped accel calibration");
			}
		}
	}
	
	// Button just released - check hold duration
	if (!btnPressed && _lastSaveBtnState) {
		Serial.printf("Save button: RELEASED (held %lu ms)\n", (unsigned long)heldTime);
		
		if (heldTime <= kSaveBtnLongPressMs) {
			// Short press: save settings
			saveSettings();
			beep(kBeepShortMs);
		}
	}
	
	_lastSaveBtnState = btnPressed;
}

void Interface::saveSettings() {
	_controller.saveSettings();
	Serial.println("Settings saved to NVS");
}

// ==================== KEYBOARD INPUT METHODS ====================

void Interface::printHelp() {
	Serial.println("\n=== Melty Controller CLI ===");
	Serial.println("Rotation:      A/D=medium, Q/E=small, Z/C=large");
	Serial.println("               (hold key to steer, release to stop)");
	Serial.println("Motor power:   +=+1%, -=-1% (0-100%)");
	Serial.println("Forward speed: W=+5%, R=-5% (0-100%)");
	Serial.println("Radius adjust: >=+0.01mm, <=-0.01mm");
	Serial.println("Motor offset:  .=+1deg, ,=-1deg");
	Serial.println("Calibrate:     X=calibrate accelerometer");
	Serial.println("Save:          S=save radius & offset to NVS");
	Serial.println("Info:          I=request info, G=request settings");
	Serial.println("Other:         H=? help, P=print status");
}

void Interface::printStatus() {
	Serial.println("\n--- Current Status ---");
	Serial.printf("Connected: %s\n", _controller.isConnected() ? "YES" : "NO");
	Serial.printf("Motor power: %.0f%%\n", (double)_controller.getMotorPower());
	Serial.printf("Forward speed: %.0f%%\n", (double)_controller.getForwardSpeed());
	Serial.printf("Base radius: %.3f mm\n", (double)_controller.getRadiusMm());
	Serial.printf("Rotation delta: %.3f mm\n", (double)_controller.getRotationDelta());
	Serial.printf("Motor offset: %.1f deg\n", (double)_controller.getMotorOffsetDeg());
	Serial.println("----------------------");
}


void Interface::checkRotateRelease() {
	const uint32_t nowMs = millis();
	
	if (_rotatingRight && (nowMs - _rotateRightLastMs) > kRotateReleaseTimeoutMs) {
		_rotatingRight = false;
		_controller.stopRotate();
	}
	
	if (_rotatingLeft && (nowMs - _rotateLeftLastMs) > kRotateReleaseTimeoutMs) {
		_rotatingLeft = false;
		_controller.stopRotate();
	}
}

void Interface::readKeyboard() {
	while (Serial.available() > 0) {
		const int c = Serial.read();
		if (c < 0) break;

		// Convert to uppercase for case insensitivity
		char ch = static_cast<char>(c);
		if (ch >= 'a' && ch <= 'z') {
			ch = static_cast<char>(ch - 'a' + 'A');
		}

		handleKey(ch);
	}
}

void Interface::handleKey(char key) {
	// Key is already uppercase

	switch (key) {
		case 'H':
		case '?':
			printHelp();
			return;

		case 'P':
			printStatus();
			return;

		// === Motor Power (like Encoder 1) ===
		
		case '+':
		case '=': {
			float newPower = _controller.getMotorPower() + 1.0f;
			_controller.setMotorPower(newPower);
			Serial.printf("Motor power: %.0f%%\n", (double)_controller.getMotorPower());
			return;
		}
		case '-': {
			float newPower = _controller.getMotorPower() - 1.0f;
			_controller.setMotorPower(newPower);
			Serial.printf("Motor power: %.0f%%\n", (double)_controller.getMotorPower());
			return;
		}

		// === Forward Speed (like Joystick Y) ===
		case 'W': {
			float newSpeed = _controller.getForwardSpeed() + 5.0f;
			_controller.setForwardSpeed(newSpeed);
			Serial.printf("Forward speed: %.0f%%\n", (double)_controller.getForwardSpeed());
			return;
		}
		case 'R': {
			float newSpeed = _controller.getForwardSpeed() - 5.0f;
			_controller.setForwardSpeed(newSpeed);
			Serial.printf("Forward speed: %.0f%%\n", (double)_controller.getForwardSpeed());
			return;
		}

		// === Radius Adjustment (like Encoder 2) ===
		case '>': {
			_controller.adjustAccelRadius(0.01f);
			Serial.printf("Base radius: %.3f mm\n", (double)_controller.getRadiusMm());
			return;
		}
		case '<': {
			_controller.adjustAccelRadius(-0.01f);
			Serial.printf("Base radius: %.3f mm\n", (double)_controller.getRadiusMm());
			return;
		}

		// === Motor Offset (like Encoder 3) ===
		case '.': {
			_controller.adjustMotorFwdOffset(1.0f);
			Serial.printf("Motor offset: %.1f deg\n", (double)_controller.getMotorOffsetDeg());
			return;
		}
		case ',': {
			_controller.adjustMotorFwdOffset(-1.0f);
			Serial.printf("Motor offset: %.1f deg\n", (double)_controller.getMotorOffsetDeg());
			return;
		}

		// === Rotation - Medium (hold to steer, releases on key up) ===
		case 'A':
			_rotateLeftLastMs = millis();
			_rotatingLeft = true;
			_controller.rotateLeft(Controller::kRotateMedium);
			Serial.printf("Rotate left (medium): delta=%.3f mm\n", (double)_controller.getRotationDelta());
			return;
		case 'D':
			_rotateRightLastMs = millis();
			_rotatingRight = true;
			_controller.rotateRight(Controller::kRotateMedium);
			Serial.printf("Rotate right (medium): delta=%.3f mm\n", (double)_controller.getRotationDelta());
			return;

		// === Rotation - Small (hold to steer, releases on key up) ===
		case 'Q':
			_rotateLeftLastMs = millis();
			_rotatingLeft = true;
			_controller.rotateLeft(Controller::kRotateSmall);
			Serial.printf("Rotate left (small): delta=%.3f mm\n", (double)_controller.getRotationDelta());
			return;
		case 'E':
			_rotateRightLastMs = millis();
			_rotatingRight = true;
			_controller.rotateRight(Controller::kRotateSmall);
			Serial.printf("Rotate right (small): delta=%.3f mm\n", (double)_controller.getRotationDelta());
			return;

		// === Rotation - Large (hold to steer, releases on key up) ===
		case 'Z':
			_rotateLeftLastMs = millis();
			_rotatingLeft = true;
			_controller.rotateLeft(Controller::kRotateLarge);
			Serial.printf("Rotate left (large): delta=%.3f mm\n", (double)_controller.getRotationDelta());
			return;
		case 'C':
			_rotateRightLastMs = millis();
			_rotatingRight = true;
			_controller.rotateRight(Controller::kRotateLarge);
			Serial.printf("Rotate right (large): delta=%.3f mm\n", (double)_controller.getRotationDelta());
			return;

		// === Save Settings ===
		case 'S':
			saveSettings();
			return;

		// === Calibration (like Button long press) ===
		case 'X':
			if (!_controller.isConnected()) {
				Serial.println("Not connected!");
				return;
			}
			_controller.calibrateAccel();
			_isCalibrating = true;
			_calibrateStartMs = millis();
			Serial.println("TX: Calibrate accel (1000ms)");
			return;

		// === Info/Settings Requests ===
		case 'I':
			if (!_controller.isConnected()) {
				Serial.println("Not connected!");
				return;
			}
			_controller.requestInfo();
			Serial.println("TX: Info request");
			return;
		case 'G':
			if (!_controller.isConnected()) {
				Serial.println("Not connected!");
				return;
			}
			_controller.requestSettings();
			Serial.println("TX: Settings request");
			return;

		default:
			return;
	}
}

// ==================== CALLBACKS ====================

void Interface::onInfoReceived(const Controller::Info& info) {
	Serial.printf("RX Info: rev/s=%.3f updates/s=%.1f signal=%d\n",
		(double)info.revPerSec, (double)info.updatesPerSec, (int)info.signalStrength);
	_lastInfo = info;
	_hasInfo = true;
}

void Interface::onSettingsReceived(const Controller::Settings& settings) {
	Serial.printf("RX Settings: radius=%.3fmm channel=%u motorOffset=%.1fdeg\n",
		(double)settings.radiusMm, (unsigned)settings.wifiChannel, (double)settings.motorOffsetDeg);
	_lastSettings = settings;
	_hasSettings = true;
}

