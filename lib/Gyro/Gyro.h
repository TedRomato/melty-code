#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <functional>

class Gyro {
public:
	// MPU6050 gyro full-scale range (FS_SEL) values.
	// These map directly to the GYRO_CONFIG FS_SEL field (0-3).
	enum class GyroRange : uint8_t {
		deg_per_sec250 = 0,
		deg_per_sec500 = 1,
		deg_per_sec1000 = 2,
		deg_per_sec2000 = 3,
	};

	// MPU6050 accel full-scale range (AFS_SEL) values.
	// These map directly to the ACCEL_CONFIG AFS_SEL field (0-3).
	enum class AccelRange : uint8_t {
		g_2 = 0,
		g_4 = 1,
		g_8 = 2,
		g_16 = 3,
	};

	// Raw, send-friendly data (matches common MPU6050 register output types)
	struct __attribute__((packed)) AccelRaw {
		int16_t x;
		int16_t y;
		int16_t z;
	};

	struct __attribute__((packed)) GyroRaw {
		int16_t x;
		int16_t y;
		int16_t z;
	};

	struct __attribute__((packed)) TempRaw {
		int16_t raw;
	};

	struct __attribute__((packed)) AllRaw {
		AccelRaw accel;
		TempRaw temp;
		GyroRaw gyro;
	};

	// Human readable (SI-ish)
	struct AccelHR {
		float x_mps2;
		float y_mps2;
		float z_mps2;
	};

	struct GyroHR {
		float x_dps;
		float y_dps;
		float z_dps;
	};

	struct TempHR {
		float c;
	};

	struct AllHR {
		AccelHR accel;
		TempHR temp;
		GyroHR gyro;
	};

	Gyro() = default;

	// Initialize MPU6050 (or compatible) over I2C.
	// If sdaPin/sclPin are -1, uses the core defaults.
	// Returns true if the device ACKs and was configured.
	bool init(int sdaPin = -1,
	          int sclPin = -1,
	          uint8_t address = 0x68,
	          uint32_t i2cHz = 400000);

	GyroRange range() const { return _range; }
	AccelRange accelRange() const { return _accelRange; }

	bool setGyroRange(GyroRange range);
	bool setAccelRange(AccelRange range);

	// Measures gyro bias for the specified duration (default 5s) and stores it.
	// Subsequent gyro reads (readGyro and readAll gyro portion) subtract this bias.
	// Returns true if at least one sample was captured.
	bool calibrate(uint32_t durationMs = 1000);

	// Measures accel bias for the specified duration (default 5s) and stores it.
	// Subsequent accel reads (readAccel and readAll accel portion) subtract X/Y bias only.
	// Z axis is intentionally left untouched.
	// Returns true if at least one sample was captured.
	// Optional callback is called periodically (~every 50ms) to allow keeping radio alive.
	using CalibrationCallback = std::function<void()>;
	bool calibrateAccel(uint32_t durationMs = 1000, CalibrationCallback periodicCallback = nullptr);

	AccelRaw readAccel();
	GyroRaw readGyro();
	TempRaw readTemp();
	AllRaw readAll();

	// Defaults to assuming ±16g.
	static AccelHR accelHR(const AccelRaw& raw);
	static AccelHR accelHR(const AccelRaw& raw, AccelRange range);
	// Defaults to assuming ±250 dps.
	static GyroHR gyroHR(const GyroRaw& raw);
	static GyroHR gyroHR(const GyroRaw& raw, GyroRange range);
	static TempHR tempHR(const TempRaw& raw);
	// Defaults to assuming ±250 dps.
	static AllHR allHR(const AllRaw& raw);
	static AllHR allHR(const AllRaw& raw, GyroRange range);

	static void printAccel(const AccelRaw& raw, Print& out = Serial);
	static void printGyro(const GyroRaw& raw, Print& out = Serial);
	static void printTemp(const TempRaw& raw, Print& out = Serial);
	static void printAll(const AllRaw& raw, Print& out = Serial);

	static void printAccel(const AccelHR& hr, Print& out = Serial);
	static void printGyro(const GyroHR& hr, Print& out = Serial);
	static void printTemp(const TempHR& hr, Print& out = Serial);
	static void printAll(const AllHR& hr, Print& out = Serial);

private:
	TwoWire* _wire = &Wire;
	uint8_t _address = 0x68;
	bool _initialized = false;
	GyroRange _range = GyroRange::deg_per_sec250;
	AccelRange _accelRange = AccelRange::g_16;

	int16_t _gyroBiasX = 0;
	int16_t _gyroBiasY = 0;
	int16_t _gyroBiasZ = 0;

	int16_t _accelBiasX = 0;
	int16_t _accelBiasY = 0;

	bool writeReg(uint8_t reg, uint8_t value);
	bool readBytes(uint8_t startReg, uint8_t* buf, size_t len);
};

// Convenience alias for app code.
using GyroRange = Gyro::GyroRange;
using AccelRange = Gyro::AccelRange;

// Convenience constants (allow passing `deg_per_sec2000` without any prefix).
static constexpr GyroRange deg_per_sec250 = GyroRange::deg_per_sec250;
static constexpr GyroRange deg_per_sec500 = GyroRange::deg_per_sec500;
static constexpr GyroRange deg_per_sec1000 = GyroRange::deg_per_sec1000;
static constexpr GyroRange deg_per_sec2000 = GyroRange::deg_per_sec2000;

static constexpr AccelRange g_2 = AccelRange::g_2;
static constexpr AccelRange g_4 = AccelRange::g_4;
static constexpr AccelRange g_8 = AccelRange::g_8;
static constexpr AccelRange g_16 = AccelRange::g_16;
