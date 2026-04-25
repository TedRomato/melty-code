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
	INFO_REQUEST = 8,
	INFO_RESPONSE = 9,
	SETTINGS_REQUEST = 10,
	SETTINGS_RESPONSE = 11,
	PING = 12,  // Heartbeat ping from controller to keep melty alive
};

struct __attribute__((packed)) ControlMsg {
	uint8_t kind; // MsgKind::CONTROL
	uint8_t _reserved1; // Previously forwardEnabled, now unused (always treated as enabled)
	uint16_t _reserved0; // padding/reserved for future flags (keeps floats aligned)
	float leftPercent;
	float rightPercent;
	float headingMultiplier;
	float motorLagDeg;
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

// Request info from melty (telemetry data)
struct __attribute__((packed)) InfoRequestMsg {
	uint8_t kind; // MsgKind::INFO_REQUEST
};

// Response with telemetry info
struct __attribute__((packed)) InfoResponseMsg {
	uint8_t kind; // MsgKind::INFO_RESPONSE
	float revPerSec;      // Rotations per second from accel
	float updatesPerSec;  // Update rate on melty
	int8_t signalStrength; // RSSI or similar
};

// Request settings from melty
struct __attribute__((packed)) SettingsRequestMsg {
	uint8_t kind; // MsgKind::SETTINGS_REQUEST
};

// Response with current settings
struct __attribute__((packed)) SettingsResponseMsg {
	uint8_t kind; // MsgKind::SETTINGS_RESPONSE
	float radiusMm;       // Base radius in mm
	uint8_t wifiChannel;  // WiFi channel
	float motorOffsetDeg; // Motor offset in degrees
};

// Heartbeat ping from controller to keep melty alive
struct __attribute__((packed)) PingMsg {
	uint8_t kind; // MsgKind::PING
};
