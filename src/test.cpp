// Bidirectional MotorDrivers test.
//
// Two ESCs are wired to GPIO6 and GPIO7.
// The motors run in opposite directions.
// Power is stepped from 0 % to 50 % by 1 %, holding each step for 5 seconds.

#include <Arduino.h>
#include "MotorDrivers.h"

namespace {
constexpr int kLeftEscPin       = 6;
constexpr int kRightEscPin      = 7;

constexpr int kEscReverseUs     = 1012;
constexpr int kEscNeutralUs     = 1488;
constexpr int kEscForwardUs     = 2020;
constexpr int kEscArmMs         = 4000;

// Set high enough so the step changes happen basically immediately.
// You are measuring steady-state current, so ramp speed is not very important here.
constexpr float kAccelPctPerSec = 200.0f;

constexpr int kMaxPowerPct = 50;
constexpr int kStepPct     = 1;

constexpr unsigned long kHoldMs = 5000;
constexpr unsigned long kTickMs = 10;

MotorDrivers motors(kLeftEscPin, kRightEscPin);

bool motorsAt(float leftTargetPct, float rightTargetPct) {
  return fabsf(motors.getLeftSpeed() - leftTargetPct) <= 0.5f &&
         fabsf(motors.getRightSpeed() - rightTargetPct) <= 0.5f;
}

void holdCurrentSpeeds(unsigned long holdMs) {
  const unsigned long endTime = millis() + holdMs;

  while (static_cast<long>(millis() - endTime) < 0) {
    motors.update();
    delay(kTickMs);
  }
}

void setOppositeDirections(float powerPct, unsigned long dwellMs) {
  float leftTarget  = +powerPct;
  float rightTarget = -powerPct;

  Serial.print(millis());
  Serial.print(" ms  set power = ");
  Serial.print(powerPct, 1);
  Serial.print(" %   L=");
  Serial.print(leftTarget, 1);
  Serial.print(" %   R=");
  Serial.print(rightTarget, 1);
  Serial.println(" %");

  // IMPORTANT:
  // This assumes your MotorDrivers library has separate left/right setters.
  motors.setLeftSpeed(leftTarget,  /*holdMs=*/0, /*acceleration=*/true);
  motors.setRightSpeed(rightTarget, /*holdMs=*/0, /*acceleration=*/true);

  while (!motorsAt(leftTarget, rightTarget)) {
    motors.update();
    delay(kTickMs);
  }

  holdCurrentSpeeds(dwellMs);
}

void stopMotors(unsigned long dwellMs) {
  Serial.println("Stopping motors...");

  motors.setLeftSpeed(0.0f,  /*holdMs=*/0, /*acceleration=*/true);
  motors.setRightSpeed(0.0f, /*holdMs=*/0, /*acceleration=*/true);

  while (!motorsAt(0.0f, 0.0f)) {
    motors.update();
    delay(kTickMs);
  }

  holdCurrentSpeeds(dwellMs);
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("=== Opposite-direction ESC power sweep ===");
  Serial.println("Left motor:  forward");
  Serial.println("Right motor: reverse");
  Serial.println("Power: 0 % to 50 %, step 1 %, 5 seconds each");

  motors.init(kEscReverseUs, kEscNeutralUs, kEscForwardUs,
              kEscArmMs, kAccelPctPerSec);

  Serial.println("ESCs armed, starting sweep.");
}

void loop() {
  for (int power = 0; power <= kMaxPowerPct; power += kStepPct) {
    setOppositeDirections(static_cast<float>(power), kHoldMs);
  }

  stopMotors(3000);

  Serial.println("--- sweep complete, repeating ---");
}