#include <Arduino.h>

#include "WirelessLink.h"
#include "Messages.h"

// ============================
// Controller (handheld) main
// ============================

#ifndef ESPNOW_CHANNEL
#define ESPNOW_CHANNEL 6
#endif

// Robot (melty) ESP32 STA MAC: 20:6E:F1:6B:8A:58
static uint8_t peerMac[6] = {0x20, 0x6E, 0xF1, 0x6B, 0x8A, 0x58};

static WirelessLink radio;

static float leftPercent = 0.0f;
static float rightPercent = 0.0f;

static float radiusMm = 2.0f;

enum Target { BOTH, LEFT, RIGHT };
static Target target = BOTH;

enum AdjustMode { ADJUST_MOTORS, ADJUST_RADIUS };
static AdjustMode adjustMode = ADJUST_MOTORS;

static bool gyroStreaming = false;
static uint16_t gyroIntervalMs = 50;

static float clampPercent(float p) {
  if (p < 0.0f) return 0.0f;
  if (p > 100.0f) return 100.0f;
  return p;
}

static const char* targetToStr(Target t) {
  switch (t) {
    case BOTH:
      return "BOTH";
    case LEFT:
      return "LEFT";
    case RIGHT:
      return "RIGHT";
    default:
      return "?";
  }
}

static void printHelp() {
  Serial.println("\n=== Controller CLI ===");
  Serial.println("Motor select:  B=both, L=left, R=right");
  Serial.println("Adjust mode:   M=motors, O=radius");
  Serial.println("Motor adjust:  +=+10%, -=-10% (selected, when in motor mode)");
  Serial.println("Radius adjust: +=+0.01mm, -=-0.01mm (when in radius mode)");
  Serial.println("Stream:        G=toggle on/off");
  Serial.println("Stream rate:   1=20ms, 2=50ms, 3=100ms");
  Serial.println("Accel calibrate:A=calibrate X/Y bias (Z untouched)");
  Serial.println("Other:         H=? help");
}

static void sendControl() {
  if (!radio.isConnected()) {
    return;
  }

  ControlMsg msg{};
  msg.kind = static_cast<uint8_t>(MsgKind::CONTROL);
  msg.leftPercent = leftPercent;
  msg.rightPercent = rightPercent;
  (void)radio.sendData(&msg, sizeof(msg));

  Serial.printf("TX Control: L=%.0f%% R=%.0f%% (target=%s)\n", msg.leftPercent, msg.rightPercent,
                targetToStr(target));
}

static void sendGyroConfig() {
  if (!radio.isConnected()) {
    return;
  }

  GyroConfigMsg msg{};
  msg.kind = static_cast<uint8_t>(MsgKind::GYRO_CONFIG);
  msg.enabled = gyroStreaming ? 1 : 0;
  msg.intervalMs = gyroIntervalMs;
  (void)radio.sendData(&msg, sizeof(msg));

  Serial.printf("TX GyroCfg: %s, %u ms\n", gyroStreaming ? "ON" : "OFF",
                (unsigned)gyroIntervalMs);
}

static void sendAccelCalibrate(uint16_t durationMs = 5000) {
  if (!radio.isConnected()) {
    Serial.println("Wireless: NOT CONNECTED");
    return;
  }

  AccelCalibrateMsg msg{};
  msg.kind = static_cast<uint8_t>(MsgKind::ACCEL_CALIBRATE);
  msg.durationMs = durationMs;
  (void)radio.sendData(&msg, sizeof(msg));

  Serial.printf("TX AccelCal: %u ms\n", (unsigned)durationMs);
}

static void sendRadiusConfig() {
  if (!radio.isConnected()) {
    return;
  }

  RadiusConfigMsg msg{};
  msg.kind = static_cast<uint8_t>(MsgKind::RADIUS_CONFIG);
  msg.radiusM = radiusMm * 1e-3f;
  (void)radio.sendData(&msg, sizeof(msg));

  Serial.printf("TX Radius: %.3f mm (%.6f m)\n", radiusMm, msg.radiusM);
}

static void handleKey(char key) {
  if (key >= 'a' && key <= 'z') {
    key = static_cast<char>(key - 'a' + 'A');
  }

  switch (key) {
    case 'H':
    case '?':
      printHelp();
      return;

    case 'B':
      target = BOTH;
      Serial.println("Selected: BOTH");
      return;
    case 'L':
      target = LEFT;
      Serial.println("Selected: LEFT");
      return;
    case 'R':
      target = RIGHT;
      Serial.println("Selected: RIGHT");
      return;

    case 'M':
      adjustMode = ADJUST_MOTORS;
      Serial.println("Adjust mode: MOTORS");
      return;
    case 'O':
      adjustMode = ADJUST_RADIUS;
      Serial.println("Adjust mode: RADIUS");
      sendRadiusConfig();
      return;

    case '+': {
      if (adjustMode == ADJUST_RADIUS) {
        radiusMm += 0.01f;
        if (radiusMm < 1.0f) radiusMm = 1.0f;
        sendRadiusConfig();
        return;
      }

      const float delta = 10.0f;
      if (target == BOTH || target == LEFT) leftPercent = clampPercent(leftPercent + delta);
      if (target == BOTH || target == RIGHT) rightPercent = clampPercent(rightPercent + delta);
      sendControl();
      return;
    }
    case '-': {
      if (adjustMode == ADJUST_RADIUS) {
        radiusMm -= 0.01f;
        if (radiusMm < 1.0f) radiusMm = 1.0f;
        sendRadiusConfig();
        return;
      }

      const float delta = 10.0f;
      if (target == BOTH || target == LEFT) leftPercent = clampPercent(leftPercent - delta);
      if (target == BOTH || target == RIGHT) rightPercent = clampPercent(rightPercent - delta);
      sendControl();
      return;
    }

    case 'G':
      gyroStreaming = !gyroStreaming;
      sendGyroConfig();
      return;

    case '1':
      gyroIntervalMs = 20;
      sendGyroConfig();
      return;
    case '2':
      gyroIntervalMs = 50;
      sendGyroConfig();
      return;
    case '3':
      gyroIntervalMs = 100;
      sendGyroConfig();
      return;

    case 'A':
      sendAccelCalibrate();
      return;

    default:
      // Ignore other keys.
      return;
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n\n=== Melty Controller ===");
  printHelp();

  radio.setOnConnect([]() {
    Serial.println("Wireless: CONNECTED");
    // Push current state on connect.
    sendControl();
    sendGyroConfig();
    sendRadiusConfig();
  });

  radio.setOnDataReceived([](const uint8_t* data, size_t len) {
    if (!data || len < 1) {
      return;
    }

    const uint8_t kind = data[0];
    if (kind == static_cast<uint8_t>(MsgKind::ACCEL_OMEGA)) {
      if (len < sizeof(AccelOmegaMsg)) {
        return;
      }

      AccelOmegaMsg msg{};
      memcpy(&msg, data, sizeof(msg));
      Serial.printf(
          "RX AccelOmega: tUs=%10lu ax=%8.2f ay=%8.2f aXY=%8.2f m/s^2 omega=%8.2f rad/s (%7.3f rev/s)\n",
          (unsigned long)msg.tUs,
          msg.ax_mps2,
          msg.ay_mps2,
          msg.aXY_mps2,
          msg.omega_rad_s,
          msg.revPerSec);
      return;
    }
  });

  const bool ok = radio.initCommunication(peerMac, ESPNOW_CHANNEL, WIFI_POWER_13dBm);
  Serial.printf("Wireless init: %s\n", ok ? "OK" : "FAIL");
}

void loop() {
  radio.update();

  while (Serial.available() > 0) {
    const int c = Serial.read();
    if (c < 0) break;
    handleKey(static_cast<char>(c));
  }
}
