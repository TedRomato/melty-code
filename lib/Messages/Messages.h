#pragma once

#include <Arduino.h>

// Shared ESP-NOW message definitions used by controller + robot.
// Keep these tightly packed and stable across both builds.

enum class MsgKind : uint8_t {
	CONTROL = 1,
	GYRO_CONFIG = 2,
	GYRO_RAW = 3,
	GYRO_CALIBRATE = 4,
	ACCEL_OMEGA = 5,
	RADIUS_CONFIG = 6,
	ACCEL_CALIBRATE = 7,
};

struct __attribute__((packed)) ControlMsg {
	uint8_t kind; // MsgKind::CONTROL
	float leftPercent;
	float rightPercent;
};

struct __attribute__((packed)) GyroConfigMsg {
	uint8_t kind; // MsgKind::GYRO_CONFIG
	uint8_t enabled; // 0/1
	uint16_t intervalMs;
};

struct __attribute__((packed)) GyroRawMsg {
	uint8_t kind; // MsgKind::GYRO_RAW
	uint32_t tUs;
	int16_t gx;
	int16_t gy;
	int16_t gz;
	float gzDps;
	float revPerSec;
};

struct __attribute__((packed)) AccelOmegaMsg {
	uint8_t kind; // MsgKind::ACCEL_OMEGA
	uint32_t tUs;
	int16_t ax;
	int16_t ay;
	int16_t az;
	float ax_mps2;
	float ay_mps2;
	float aXY_mps2;
	float omega_rad_s;
	float revPerSec;
};

struct __attribute__((packed)) GyroCalibrateMsg {
	uint8_t kind; // MsgKind::GYRO_CALIBRATE
	uint16_t durationMs;
};

struct __attribute__((packed)) AccelCalibrateMsg {
	uint8_t kind; // MsgKind::ACCEL_CALIBRATE
	uint16_t durationMs;
};

struct __attribute__((packed)) RadiusConfigMsg {
	uint8_t kind; // MsgKind::RADIUS_CONFIG
	float radiusM;
};
