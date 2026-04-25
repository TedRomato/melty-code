// MotorDrivers.h
#pragma once

#include <ESP32Servo.h>

class MotorDrivers {
public:
  // constructor with pins
  MotorDrivers(int leftPin, int rightPin);

  // minUs/maxUs = run range
  // armMs = how long to arm at 1000 us
  // accelPercentPerSec = max change in % per second
  void init(int minUs, int maxUs, int armMs, float accelPercentPerSec);

  // call frequently in loop() to handle acceleration & timed speeds
  void update();

  // Set both motors to the same speed (0–100 %).
  // holdMs = 0 → stay at that target
  // holdMs > 0 → hold for that long, then ramp back to 0 %
  // acceleration = true  -> ramp via accelPercentPerSec in update()
  // acceleration = false -> apply instantly
  void setSpeed(float percent, unsigned long holdMs = 0, bool acceleration = false);

  // Returns the currently applied speed (0–100 %) for each motor.
  float getLeftSpeed() const;
  float getRightSpeed() const;

  // Returns true only when both motors' current speeds are exactly 0%.
  bool areMotorsOff() const;

  void setLeftSpeed(float percent, unsigned long holdMs = 0, bool acceleration = false);
  void setRightSpeed(float percent, unsigned long holdMs = 0, bool acceleration = false);
  void sweepSpeed(int fromUs, int toUs, unsigned long durationMs);

private:
  Servo _leftEsc;
  Servo _rightEsc;

  int _leftPin;
  int _rightPin;

  int _minUs;
  int _maxUs;

  float _accelRate; // percent per second

  float _leftCurrent; // current % 0–100
  float _rightCurrent;
  float _leftTarget;  // target % 0–100
  float _rightTarget;

  unsigned long _leftHoldUntil;  // 0 = no timer
  unsigned long _rightHoldUntil; // 0 = no timer

  unsigned long _lastUpdateMs;

  int percentToUs(float percent) const;
  void stepMotor(float &current, float &target, unsigned long nowMs, float dt);
};
