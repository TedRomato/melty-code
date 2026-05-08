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

// malty ESP32 STA MAC: 10:00:3B:BC:73:D8 na kontroleru je 10:B4:1D:0D:09:94
static uint8_t peerMac[6] = {0x10, 0xB4, 0x1D, 0x0D, 0x09, 0x94};

static WirelessLink radio;

// TODO: Set to your ESC signal pins.
static constexpr int kLeftEscPin = 6;
static constexpr int kRightEscPin = 7;

// ESC pulse range; tune for your ESCs.
static constexpr int kEscReverseUs = 1012;
static constexpr int kEscNeutralUs = 1488;
static constexpr int kEscForwardUs = 2020;
static constexpr int kEscArmMs = 10000;
static constexpr float kEscAccelPercentPerSec = 250.0f;

static MotorDrivers motors(kLeftEscPin, kRightEscPin);
static Gyro gyro;

static bool gyroStreaming = false;
static uint16_t gyroIntervalMs = 50;
static uint32_t nextGyroSendMs = 0;

static uint32_t lastGyroUs = 0;
static float angleDeg = 0.0f;

static float baseLeftPercent = 0.0f;
static float baseRightPercent = 0.0f;
static float headingMultiplier = 0.0f;
static float motorLagDeg = 0.0f;

// Radius used for accel-derived omega calculation.
// Stored in meters; controller adjusts in mm.
static float radiusM = 0.002f;

// Latest accel Z reading (raw) – used to detect orientation flip.
static int16_t lastAccelZRaw = 0;

// Telemetry tracking
static float lastOmegaRadS = 0.0f;
static uint32_t loopCount = 0;
static uint32_t lastLoopCountResetMs = 0;
static float updatesPerSec = 0.0f;

// Periodic info send
static constexpr uint32_t kInfoSendIntervalMs = 250;
static uint32_t lastInfoSendMs = 0;

// Calibration state
static bool calibrationInProcess = false;
static bool calibrated = false;

static constexpr float kPi = 3.14159265358979323846f;


static constexpr uint8_t kLedPin = 3;
static constexpr uint8_t kLedOn = HIGH;
static constexpr uint8_t kLedOff = LOW;
static bool ledIsOn = false;
static uint32_t ledOffAtMs = 0;

static float clampPercent(float p) {
  if (p < -100.0f) return -100.0f;
  if (p > 100.0f) return 100.0f;
  return p;
}

static float clampHeadingMultiplier(float m) {
  if (m < 0.0f) return 0.0f;
  if (m > 100.0f) return 100.0f;
  return m;
}

static float degToRad(float deg) {
  return deg * (kPi / 180.0f);
}

static void updateMotorTargetsFromHeading() {
  const float baseL = clampPercent(baseLeftPercent);
  const float baseR = clampPercent(baseRightPercent);


  const float mult = clampHeadingMultiplier(headingMultiplier);
  const float flipOffset = (lastAccelZRaw < 0) ? 180.0f : 0.0f;
  const float motor1AngleDeg = angleDeg - 90.0f + motorLagDeg + flipOffset;
  const float motor2AngleDeg = angleDeg + 90.0f + motorLagDeg + flipOffset;

  const float l = baseL + cosf(degToRad(motor1AngleDeg)) * mult;
  const float r = baseR + cosf(degToRad(motor2AngleDeg)) * mult;
  // Serial.printf("Heading: %.1f deg, motor angles: %.1f deg (L), %.1f deg (R), mult=%.2f, baseL=%.2f, baseR=%.2f, l=%.2f, r=%.2f\n",
                // angleDeg, motor1AngleDeg, motor2AngleDeg, mult, baseL, baseR, l, r);
  motors.setLeftSpeed(clampPercent(l));
  motors.setRightSpeed(clampPercent(r));
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
  if (!calibrated) {
    // Ignore motor power requests until calibration is complete
    return;
  }
  headingMultiplier = msg.headingMultiplier;
  motorLagDeg = msg.motorLagDeg;
  Serial.printf("L=%.2f, R=%.2f\n", msg.leftPercent, msg.rightPercent);
  baseLeftPercent = clampPercent(msg.leftPercent);
  baseRightPercent = clampPercent(msg.rightPercent);

  // Apply immediately (loop() will continue to update targets every iteration).
  updateMotorTargetsFromHeading();
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
  if (calibrated) {
    Serial.println("Accel calibrate: IGNORED (already calibrated)");
    return;
  }

  if (calibrationInProcess) {
    Serial.println("Accel calibrate: IGNORED (already in progress)");
    return;
  }

  const uint32_t nowMs = millis();
  flashLedBrief(nowMs, 1000);

  if (!motors.areMotorsOff()) {
    Serial.println("Accel calibrate: REFUSED (motors not off)");
    return;
  }

  calibrationInProcess = true;
  const bool prevStreaming = gyroStreaming;
  gyroStreaming = false;
  nextGyroSendMs = nowMs;

  const uint32_t durationMs = (msg.durationMs == 0) ? 1000u : static_cast<uint32_t>(msg.durationMs);
  Serial.printf("Accel calibrate: start (%lu ms)\n", (unsigned long)durationMs);
  
  // Pass callback to keep radio alive during blocking calibration
  const bool ok = gyro.calibrateAccel(durationMs, []() {
    radio.update();
  });
  Serial.printf("Accel calibrate: %s\n", ok ? "OK" : "FAIL");

  if (ok) {
    calibrated = true;
  }

  gyroStreaming = prevStreaming;
  nextGyroSendMs = millis();
  calibrationInProcess = false;
}

static void handleRadiusConfig(const RadiusConfigMsg& msg) {
  // Guard against bad values (avoid divide-by-zero).
  if (!(msg.radiusM > 0.0f)) {
    Serial.println("Radius config: REFUSED (non-positive)");
    return;
  }

  radiusM = msg.radiusM;
  // Serial.printf("Radius set: %.3f mm (%.6f m)\n", radiusM * 1000.0f, radiusM);
}

static void sendInfo() {
  if (!radio.isConnected()) return;
  
  InfoResponseMsg msg{};
  msg.kind = static_cast<uint8_t>(MsgKind::INFO_RESPONSE);
  msg.revPerSec = lastOmegaRadS / (2.0f * kPi);
  msg.updatesPerSec = updatesPerSec;
  msg.signalStrength = 0; // TODO: Get actual RSSI if available
  
  radio.sendData(&msg, sizeof(msg));
}

static void handleSettingsRequest() {
  if (!radio.isConnected()) return;
  
  SettingsResponseMsg msg{};
  msg.kind = static_cast<uint8_t>(MsgKind::SETTINGS_RESPONSE);
  msg.radiusMm = radiusM * 1000.0f;
  msg.wifiChannel = ESPNOW_CHANNEL;
  msg.motorOffsetDeg = motorLagDeg;
  
  radio.sendData(&msg, sizeof(msg));
  Serial.printf("TX Settings: radius=%.3fmm channel=%u offset=%.1fdeg\n",
    (double)msg.radiusMm, (unsigned)msg.wifiChannel, (double)msg.motorOffsetDeg);
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
  delay(1000);
  Serial.println("\n\n=== Melty Robot ===");

  pinMode(kLedPin, OUTPUT);
  digitalWrite(kLedPin, kLedOff);

  // IMU init (I2C @ 400kHz)
  const bool gyroOk = gyro.init(8, 9, 0x68, 400000);
  Serial.printf("IMU init: %s\n", gyroOk ? "OK" : "FAIL");
  if (gyroOk) {
    (void)gyro.setGyroRange(deg_per_sec2000);
    (void)gyro.setAccelRange(g_16);
  }

  // Motors
  Serial.println("\nInitializing motors...");  
  motors.init(kEscReverseUs, kEscNeutralUs, kEscForwardUs, kEscArmMs, kEscAccelPercentPerSec);
  Serial.println("\nMotors initialized.");
  delay(2000); // wait a bit before spinning
  motors.setSpeed(50.0f, 4000, false); // test spin for 2s
  motors.update();

  // Wireless
  radio.setOnConnect([]() {
    Serial.println("Wireless: CONNECTED");
  });

  radio.setOnDataReceived([](const uint8_t* data, size_t len) {
    if (!data || len < 1) {
      return;
    }


    const uint8_t kind = data[0];
    if(kind != 1) {

      // Serial.printf("Received %d\n", kind);
    }
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
      Serial.printf("Calibrating\n");
      handleAccelCalibrate(msg);
      return;
    }

    if (kind == static_cast<uint8_t>(MsgKind::SETTINGS_REQUEST)) {
      handleSettingsRequest();
      return;
    }

    if (kind == static_cast<uint8_t>(MsgKind::PING)) {
      // Ping is just for keeping the connection alive, no action needed
      return;
    }
  });
  
  // Configure connection loss detection - kills motors when connection is lost
  radio.setConnectionLostTimeout(5000);  // 5 seconds without data = connection lost
  radio.setOnConnectionLost([]() {
    Serial.println("Connection lost to controller - killing motors!");
    baseLeftPercent = 0.0f;
    baseRightPercent = 0.0f;
    headingMultiplier = 0.0f;
  });

  const bool radioOk = radio.initCommunication(peerMac, ESPNOW_CHANNEL, WIFI_POWER_13dBm);
  Serial.printf("Wireless init: %s\n", radioOk ? "OK" : "FAIL");
}

void loop() {
  const uint32_t nowMs = millis();
  updateLed(nowMs);

  // Track loop rate for telemetry
  loopCount++;
  if (nowMs - lastLoopCountResetMs >= 1000) {
    updatesPerSec = static_cast<float>(loopCount);
    loopCount = 0;
    lastLoopCountResetMs = nowMs;
  }

  radio.update();

  // Send info periodically
  if (radio.isConnected() && (nowMs - lastInfoSendMs >= kInfoSendIntervalMs)) {
    lastInfoSendMs = nowMs;
    sendInfo();
  }

  // High-rate sensor loop (as fast as the loop can manage).
  const uint32_t nowUs = micros();
  if (lastGyroUs == 0) {
    lastGyroUs = nowUs;
  } else {
    const uint32_t dtUs = nowUs - lastGyroUs;
    lastGyroUs = nowUs;
    if (dtUs != 0) {
      const float dt = static_cast<float>(dtUs) * 1e-6f;

      // Accel-derived omega:
      // a = sqrt(ax^2 + ay^2)
      // omega = sqrt(a / R)
      const Gyro::AccelRaw aRaw = gyro.readAccel();
      lastAccelZRaw = aRaw.z;
      const Gyro::AccelHR a = Gyro::accelHR(aRaw, gyro.accelRange());
      const float aXY_mps2 = sqrtf((a.x_mps2 * a.x_mps2) + (a.y_mps2 * a.y_mps2));

      float omega_rad_s = 0.0f;
      if (radiusM > 0.0f && aXY_mps2 >= 0.0f) {
        omega_rad_s = sqrtf(aXY_mps2 / radiusM);
      }
      
      // Store for telemetry
      lastOmegaRadS = omega_rad_s;

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
  }

  // Update motor targets every iteration.
  updateMotorTargetsFromHeading();
  motors.update();
}