#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>

#include "Controller.h"

// Unified Interface for Melty Controller
//
// Combines keyboard (serial) input AND hardware input (OLED, joystick, encoders).
// Both input methods work simultaneously for debugging.
//
// Keyboard mappings (mimic hardware):
//   Joystick X (rotation): A=left, D=right (hold for continuous)
//                          Q/E=small, Z/C=large, S=stop
//   Joystick Y (forward):  W=+5%, R=-5%
//   Encoder 1 (power):     +/- = ±1%
//   Encoder 2 (radius):    >/< = ±0.01mm
//   Encoder 3 (offset):    ./,  = ±1 deg
//   Button (calibrate):    X = calibrate accelerometer
//   Info requests:         I = info, G = settings
//   Other:                 H/? = help, P = print status

class Interface {
public:
	// Pin configuration struct (optional for keyboard-only mode)
	struct Config {
		// I2C pins for OLED
		int8_t i2cSda;
		int8_t i2cScl;
		uint8_t oledAddr;
		
		// Joystick analog pins
		int8_t joystickXPin;   // ADC pin for X axis (rotation)
		int8_t joystickYPin;   // ADC pin for Y axis (forward speed)
		int8_t joystickBtnPin; // Button pin (for calibration trigger)
		
		// Encoder 1: Motor Power
		int8_t enc1PinA;
		int8_t enc1PinB;
		
		// Encoder 2: Radius
		int8_t enc2PinA;
		int8_t enc2PinB;
		
		// Encoder 3: Motor Offset
		int8_t enc3PinA;
		int8_t enc3PinB;
		
		// Save button pin
		int8_t saveBtnPin;
		int8_t buzzerPin;
		
		// Enable hardware inputs (set false for keyboard-only)
		bool enableHardware;
		
		// Default constructor with sensible defaults
		Config() : 
			i2cSda(5), i2cScl(6), oledAddr(0x3C),
			joystickXPin(0), joystickYPin(1), joystickBtnPin(2),
			enc1PinA(3), enc1PinB(4),
			enc2PinA(7), enc2PinB(8),
			enc3PinA(9), enc3PinB(10),
			saveBtnPin(-1),
			buzzerPin(-1),
			enableHardware(true) {}
	};

	explicit Interface(Controller& controller, const Config& config = Config{});

	void begin();
	void update();

private:
	Controller& _controller;
	Config _config;
	
	// OLED display (72x40 with offsets 33,12)
	U8G2_SSD1306_72X40_ER_F_HW_I2C* _display = nullptr;
	bool _displayInitialized = false;
	
	// Calibration display state
	bool _isCalibrating = false;
	uint32_t _calibrateStartMs = 0;
	static constexpr uint32_t kCalibrateDurationMs = 1000;
	
	// Display refresh timing100
	static constexpr uint32_t kDisplayRefreshMs = 100;
	static constexpr uint32_t kInfoRequestMs = 1000;
	uint32_t _lastDisplayRefreshMs = 0;
	uint32_t _lastInfoRequestMs = 0;
	
	// Cached telemetry data
	Controller::Info _lastInfo{};
	Controller::Settings _lastSettings{};
	bool _hasInfo = false;
	bool _hasSettings = false;
	
	// Joystick state
	static constexpr int kJoystickCenter = 2048;  // 12-bit ADC center
	static constexpr int kJoystickDeadzone = 50;   // +/- deadzone around center
	// Thresholds as percentage of max range (0.0 to 1.0)
	static constexpr float kJoystickSmallThresholdPct = 0.5f;   // 40% of range
	static constexpr float kJoystickMediumThresholdPct = 0.8f;  // 80% of range
	int _lastJoystickX = kJoystickCenter;
	int _lastJoystickY = kJoystickCenter;
	float _targetForwardSpeed = 0.0f;
	
	// Encoder state
	volatile int32_t _enc1Count = 0;
	volatile int32_t _enc2Count = 0;
	volatile int32_t _enc3Count = 0;
	int32_t _lastEnc1Count = 0;
	int32_t _lastEnc2Count = 0;
	int32_t _lastEnc3Count = 0;
	
	// Encoder acceleration state
	// After 3 consecutive fast rotations (<100ms apart), increments are boosted
	static constexpr uint32_t kEncoderAccelTimeoutMs = 100;
	static constexpr int kEncoderAccelThreshold = 2;     // Fast rotations needed to activate
	static constexpr int kEncoderAccelMultiplier = 4;    // Multiplier when accelerated
	uint32_t _enc1LastEventMs = 0;
	uint32_t _enc2LastEventMs = 0;
	uint32_t _enc3LastEventMs = 0;
	int _enc1FastCount = 0;  // Consecutive fast rotations
	int _enc2FastCount = 0;
	int _enc3FastCount = 0;
	bool _enc1AccelActive = false;  // Acceleration mode active
	bool _enc2AccelActive = false;
	bool _enc3AccelActive = false;
	int _enc1AccelToggle = 0;  // For "every other" counting
	int _enc2AccelToggle = 0;
	int _enc3AccelToggle = 0;

	// Encoder last pin states
	uint8_t _enc1LastState = 0;
	uint8_t _enc2LastState = 0;
	uint8_t _enc3LastState = 0;
	
	// Joystick button state
	bool _lastBtnState = false;
	uint32_t _btnPressedMs = 0;
	static constexpr uint32_t kLongPressMs = 1000;
	
	// Save button state
	bool _lastSaveBtnState = false;
	uint32_t _saveBtnPressedMs = 0;
	static constexpr uint32_t kSaveBtnLongPressMs = 750;
	static constexpr uint32_t kBeepShortMs = 80;
	static constexpr uint32_t kBeepLongMs = 300;
	
	// Buzzer state
	bool _buzzerOn = false;
	uint32_t _buzzerOffAtMs = 0;
	
	// Joystick center calibration (during save button hold)
	int32_t _joystickCalibSumX = 0;
	int32_t _joystickCalibSumY = 0;
	int32_t _joystickCalibCount = 0;
	int _joystickCenterX = kJoystickCenter;
	int _joystickCenterY = kJoystickCenter;
	
	// Keyboard rotation key hold tracking
	static constexpr uint32_t kRotateReleaseTimeoutMs = 150;
	uint32_t _rotateRightLastMs = 0;
	uint32_t _rotateLeftLastMs = 0;
	bool _rotatingRight = false;
	bool _rotatingLeft = false;
	
	// ===== Display methods =====
	void initDisplay();
	void refreshDisplay();
	void drawStatusScreen();
	
	// ===== Hardware input methods =====
	void readJoystick();
	void readEncoders();
	void readButton();
	void readSaveButton();
	void pollEncoder(int8_t pinA, int8_t pinB, uint8_t& lastState, volatile int32_t& count);
	void applyJoystickRotation(int xValue);
	void applyJoystickForward(int yValue);
	int32_t calculateAcceleratedSteps(int32_t rawSteps, uint32_t nowMs,
		uint32_t& lastEventMs, int& fastCount, bool& accelActive, int& accelToggle);
	void beep(uint32_t durationMs);
	void updateBuzzer(uint32_t nowMs);
	
	// ===== Keyboard input methods =====
	void readKeyboard();
	void handleKey(char key);
	void checkRotateRelease();
	void printHelp();
	void printStatus();
	void saveSettings();
	
	// ===== Callbacks =====
	void onInfoReceived(const Controller::Info& info);
	void onSettingsReceived(const Controller::Settings& settings);
};

