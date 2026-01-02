#include <Arduino.h>
#include <Wire.h>

#include "Gyro.h"
#include "MotorDrivers.h"
#include "WirelessLink.h"
#include "Messages.h"

// ============================
// Robot (melty brain) main
// ============================

#ifndef ESPNOW_CHANNEL
#define ESPNOW_CHANNEL 6
#endif

// Controller ESP32 STA MAC: 1C:DB:D4:C4:67:40
static uint8_t peerMac[6] = {0x1C, 0xDB, 0xD4, 0xC4, 0x67, 0x40};

static WirelessLink radio;

// TODO: Set to your ESC signal pins.
static constexpr int kLeftEscPin = 4;
static constexpr int kRightEscPin = 3;

// ESC pulse range; tune for your ESCs.
static constexpr int kEscMinUs = 1000;
static constexpr int kEscMaxUs = 1750;
static constexpr int kEscArmMs = 10000;
static constexpr float kEscAccelPercentPerSec = 250.0f;

static MotorDrivers motors(kLeftEscPin, kRightEscPin);
static Gyro gyro;

static bool gyroStreaming = false;
static uint16_t gyroIntervalMs = 50;
static uint32_t nextGyroSendMs = 0;

static uint32_t lastGyroUs = 0;
static float angleDeg = 0.0f;

// Radius used for accel-derived omega calculation.
// Stored in meters; controller adjusts in mm.
static float radiusM = 0.002f;

static constexpr float kPi = 3.14159265358979323846f;


static constexpr uint8_t kLedPin = 1;
static constexpr uint8_t kLedOn = HIGH;
static constexpr uint8_t kLedOff = LOW;
static bool ledIsOn = false;
static uint32_t ledOffAtMs = 0;

static float clampPercent(float p) {
  if (p < 0.0f) return 0.0f;
  if (p > 100.0f) return 100.0f;
  return p;
}

static void flashLedBrief(uint32_t nowMs, uint32_t durationMs = 2) {
  ledIsOn = true;
  ledOffAtMs = nowMs + durationMs;
  digitalWrite(kLedPin, kLedOn);
}

static void updateLed(uint32_t nowMs) {
  if (ledIsOn && static_cast<int32_t>(nowMs - ledOffAtMs) >= 0) {
    ledIsOn = false;
    digitalWrite(kLedPin, kLedOff);
  }
}

static void handleControl(const ControlMsg& msg) {
  const float l = clampPercent(msg.leftPercent);
  const float r = clampPercent(msg.rightPercent);
  motors.setLeftSpeed(l);
  motors.setRightSpeed(r);
}

static void handleGyroConfig(const GyroConfigMsg& msg) {
  gyroStreaming = (msg.enabled != 0);
  const uint16_t requested = msg.intervalMs;
  gyroIntervalMs = (requested == 0) ? 50 : requested;
  nextGyroSendMs = millis();

  Serial.printf("Gyro streaming: %s, %u ms\n", gyroStreaming ? "ON" : "OFF",
                (unsigned)gyroIntervalMs);
}

static void handleAccelCalibrate(const AccelCalibrateMsg& msg) {
  const uint32_t nowMs = millis();
  flashLedBrief(nowMs, 60);

  if (!motors.areMotorsOff()) {
    Serial.println("Accel calibrate: REFUSED (motors not off)");
    return;
  }

  const bool prevStreaming = gyroStreaming;
  gyroStreaming = false;
  nextGyroSendMs = nowMs;

  const uint32_t durationMs = (msg.durationMs == 0) ? 5000u : static_cast<uint32_t>(msg.durationMs);
  Serial.printf("Accel calibrate: start (%lu ms)\n", (unsigned long)durationMs);
  const bool ok = gyro.calibrateAccel(durationMs);
  Serial.printf("Accel calibrate: %s\n", ok ? "OK" : "FAIL");

  gyroStreaming = prevStreaming;
  nextGyroSendMs = millis();
}

static void handleRadiusConfig(const RadiusConfigMsg& msg) {
  // Guard against bad values (avoid divide-by-zero).
  if (!(msg.radiusM > 0.0f)) {
    Serial.println("Radius config: REFUSED (non-positive)");
    return;
  }

  radiusM = msg.radiusM;
  Serial.printf("Radius set: %.3f mm (%.6f m)\n", radiusM * 1000.0f, radiusM);
}

static void sendAccelOmega(const Gyro::AccelRaw& raw,
              float ax_mps2,
              float ay_mps2,
              float aXY_mps2,
              float omega_rad_s) {
  if (!radio.isConnected() || !gyroStreaming) {
    return;
  }

  const uint32_t nowMs = millis();
  if (static_cast<int32_t>(nowMs - nextGyroSendMs) < 0) {
    return;
  }
  nextGyroSendMs = nowMs + gyroIntervalMs;

  AccelOmegaMsg msg{};
  msg.kind = static_cast<uint8_t>(MsgKind::ACCEL_OMEGA);
  msg.tUs = micros();
  msg.ax = raw.x;
  msg.ay = raw.y;
  msg.az = raw.z;
  msg.ax_mps2 = ax_mps2;
  msg.ay_mps2 = ay_mps2;
  msg.aXY_mps2 = aXY_mps2;
  msg.omega_rad_s = omega_rad_s;
  msg.revPerSec = omega_rad_s / (2.0f * kPi);
  (void)radio.sendData(&msg, sizeof(msg));
}

void setup() {
  Serial.begin(115200);
  delay(5000);
  Serial.println("\n\n=== Melty Robot ===");

  pinMode(kLedPin, OUTPUT);
  digitalWrite(kLedPin, kLedOff);

  // IMU init (I2C @ 400kHz)
  const bool gyroOk = gyro.init(8, 6, 0x68, 400000);
  Serial.printf("IMU init: %s\n", gyroOk ? "OK" : "FAIL");
  if (gyroOk) {
    (void)gyro.setGyroRange(deg_per_sec2000);
    (void)gyro.setAccelRange(g_16);
  }

  // Motors
  Serial.println("\nInitializing motors...");  
  motors.init(kEscMinUs, kEscMaxUs, kEscArmMs, kEscAccelPercentPerSec);
  Serial.println("\nMotors initialized.");

  // Wireless
  radio.setOnConnect([]() {
    Serial.println("Wireless: CONNECTED");
  });

  radio.setOnDataReceived([](const uint8_t* data, size_t len) {
    if (!data || len < 1) {
      return;
    }

    const uint8_t kind = data[0];
    if (kind == static_cast<uint8_t>(MsgKind::CONTROL)) {
      if (len < sizeof(ControlMsg)) return;
      ControlMsg msg{};
      memcpy(&msg, data, sizeof(msg));
      handleControl(msg);
      return;
    }

    if (kind == static_cast<uint8_t>(MsgKind::GYRO_CONFIG)) {
      if (len < sizeof(GyroConfigMsg)) return;
      GyroConfigMsg msg{};
      memcpy(&msg, data, sizeof(msg));
      handleGyroConfig(msg);
      return;
    }

    if (kind == static_cast<uint8_t>(MsgKind::RADIUS_CONFIG)) {
      if (len < sizeof(RadiusConfigMsg)) return;
      RadiusConfigMsg msg{};
      memcpy(&msg, data, sizeof(msg));
      handleRadiusConfig(msg);
      return;
    }

    if (kind == static_cast<uint8_t>(MsgKind::ACCEL_CALIBRATE)) {
      if (len < sizeof(AccelCalibrateMsg)) return;
      AccelCalibrateMsg msg{};
      memcpy(&msg, data, sizeof(msg));
      handleAccelCalibrate(msg);
      return;
    }
  });

  const bool radioOk = radio.initCommunication(peerMac, ESPNOW_CHANNEL, WIFI_POWER_13dBm);
  Serial.printf("Wireless init: %s\n", radioOk ? "OK" : "FAIL");
}

void loop() {
  const uint32_t nowMs = millis();
  updateLed(nowMs);

  radio.update();
  motors.update();

  // High-rate sensor loop (as fast as the loop can manage).
  const uint32_t nowUs = micros();
  if (lastGyroUs == 0) {
    lastGyroUs = nowUs;
    return;
  }

  const uint32_t dtUs = nowUs - lastGyroUs;
  lastGyroUs = nowUs;
  if (dtUs == 0) {
    return;
  }

  const float dt = static_cast<float>(dtUs) * 1e-6f;

  // Accel-derived omega:
  // a = sqrt(ax^2 + ay^2)
  // omega = sqrt(a / R)
  const Gyro::AccelRaw aRaw = gyro.readAccel();
  const Gyro::AccelHR a = Gyro::accelHR(aRaw, gyro.accelRange());
  const float aXY_mps2 = sqrtf((a.x_mps2 * a.x_mps2) + (a.y_mps2 * a.y_mps2));

  float omega_rad_s = 0.0f;
  if (radiusM > 0.0f && aXY_mps2 >= 0.0f) {
    omega_rad_s = sqrtf(aXY_mps2 / radiusM);
  }

  const float omega_deg_s = omega_rad_s * (180.0f / kPi);

  angleDeg += omega_deg_s * dt;
  if (angleDeg >= 360.0f) {
    flashLedBrief(nowMs);
    while (angleDeg >= 360.0f) {
      angleDeg -= 360.0f;
    }
  }

  // Only send at the configured interval.
  if (radio.isConnected() && gyroStreaming) {
    sendAccelOmega(aRaw, a.x_mps2, a.y_mps2, aXY_mps2, omega_rad_s);
  }
}