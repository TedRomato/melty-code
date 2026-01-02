#include "Gyro.h"

namespace {
// MPU6050 registers
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t REG_TEMP_OUT_H = 0x41;
constexpr uint8_t REG_GYRO_XOUT_H = 0x43;
constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t REG_WHO_AM_I = 0x75;

constexpr float kG = 9.80665f;

float accelLSBPerGForRange(Gyro::AccelRange range) {
	switch (range) {
		case Gyro::AccelRange::g_2:
			return 16384.0f;
		case Gyro::AccelRange::g_4:
			return 8192.0f;
		case Gyro::AccelRange::g_8:
			return 4096.0f;
		case Gyro::AccelRange::g_16:
			return 2048.0f;
		default:
			return 2048.0f;
	}
}

float gyroLSBPerDPSForRange(Gyro::GyroRange range) {
	switch (range) {
		case Gyro::GyroRange::deg_per_sec250:
			return 131.0f;
		case Gyro::GyroRange::deg_per_sec500:
			return 65.5f;
		case Gyro::GyroRange::deg_per_sec1000:
			return 32.8f;
		case Gyro::GyroRange::deg_per_sec2000:
			return 16.4f;
		default:
			return 131.0f;
	}
}
} // namespace

bool Gyro::init(int sdaPin, int sclPin, uint8_t address, uint32_t i2cHz) {
	_address = address;
	_wire = &Wire;
	_range = GyroRange::deg_per_sec250;
	_accelRange = AccelRange::g_16;

	if (sdaPin >= 0 && sclPin >= 0) {
		_wire->begin(sdaPin, sclPin);
	} else {
		_wire->begin();
	}
	_wire->setClock(i2cHz);

	// Probe
	_wire->beginTransmission(_address);
	const uint8_t err = _wire->endTransmission(true);
	if (err != 0) {
		_initialized = false;
		return false;
	}

	// Optional WHO_AM_I read (best-effort, don't fail init on mismatch)
	uint8_t who = 0;
	(void)readBytes(REG_WHO_AM_I, &who, 1);

	// Wake up (clear sleep bit)
	if (!writeReg(REG_PWR_MGMT_1, 0x00)) {
		_initialized = false;
		return false;
	}
	delay(50);

	// Configure to known defaults:
	// accel ±16g, gyro ±250 dps.
	if (!setAccelRange(_accelRange)) {
		_initialized = false;
		return false;
	}
	// if (!setGyroRange(_range)) {
	// 	_initialized = false;
	// 	return false;
	// }

	_initialized = true;
	_gyroBiasX = 0;
	_gyroBiasY = 0;
	_gyroBiasZ = 0;
	_accelBiasX = 0;
	_accelBiasY = 0;
	return true;
}

bool Gyro::setGyroRange(GyroRange range) {
	// GYRO_CONFIG FS_SEL bits are [4:3], value 0-3.
	const uint8_t fs_sel = static_cast<uint8_t>(range) & 0x03;
	if (!writeReg(REG_GYRO_CONFIG, static_cast<uint8_t>(fs_sel << 3))) {
		return false;
	}
	_range = range;
	return true;
}

bool Gyro::setAccelRange(AccelRange range) {
	const uint8_t afs_sel = static_cast<uint8_t>(range) & 0x03;

	// Preserve self-test bits (7:5), change only bits 4:3
	uint8_t reg = 0;
	if (!readBytes(REG_ACCEL_CONFIG, &reg, 1)) return false;
	reg = (reg & 0b11100111) | (afs_sel << 3); // clear [4:3], set new
	if (!writeReg(REG_ACCEL_CONFIG, reg)) return false;

	_accelRange = range;
	return true;
}

bool Gyro::calibrate(uint32_t durationMs) {
	if (!_initialized) {
		_gyroBiasX = 0;
		_gyroBiasY = 0;
		_gyroBiasZ = 0;
		return false;
	}

	// Reset bias before measuring.
	_gyroBiasX = 0;
	_gyroBiasY = 0;
	_gyroBiasZ = 0;

	int64_t sumX = 0;
	int64_t sumY = 0;
	int64_t sumZ = 0;
	uint32_t samples = 0;

	const uint32_t start = millis();
	while ((millis() - start) < durationMs) {
		uint8_t buf[6] = {0};
		if (readBytes(REG_GYRO_XOUT_H, buf, sizeof(buf))) {
			const int16_t x = static_cast<int16_t>((static_cast<uint16_t>(buf[0]) << 8) | static_cast<uint16_t>(buf[1]));
			const int16_t y = static_cast<int16_t>((static_cast<uint16_t>(buf[2]) << 8) | static_cast<uint16_t>(buf[3]));
			const int16_t z = static_cast<int16_t>((static_cast<uint16_t>(buf[4]) << 8) | static_cast<uint16_t>(buf[5]));
			sumX += x;
			sumY += y;
			sumZ += z;
			samples++;
		}

		// Don't hammer the bus; ~200 Hz max.
		delay(5);
	}

	if (samples == 0) {
		_gyroBiasX = 0;
		_gyroBiasY = 0;
		_gyroBiasZ = 0;
		return false;
	}

	_gyroBiasX = static_cast<int16_t>(sumX / static_cast<int64_t>(samples));
	_gyroBiasY = static_cast<int16_t>(sumY / static_cast<int64_t>(samples));
	_gyroBiasZ = static_cast<int16_t>(sumZ / static_cast<int64_t>(samples));
	return true;
}

bool Gyro::calibrateAccel(uint32_t durationMs) {
	if (!_initialized) {
		_accelBiasX = 0;
		_accelBiasY = 0;
		return false;
	}

	// Reset bias before measuring.
	_accelBiasX = 0;
	_accelBiasY = 0;

	int64_t sumX = 0;
	int64_t sumY = 0;
	uint32_t samples = 0;

	const uint32_t start = millis();
	while ((millis() - start) < durationMs) {
		uint8_t buf[6] = {0};
		if (readBytes(REG_ACCEL_XOUT_H, buf, sizeof(buf))) {
			const int16_t x = static_cast<int16_t>((static_cast<uint16_t>(buf[0]) << 8) | static_cast<uint16_t>(buf[1]));
			const int16_t y = static_cast<int16_t>((static_cast<uint16_t>(buf[2]) << 8) | static_cast<uint16_t>(buf[3]));
			sumX += x;
			sumY += y;
			samples++;
		}

		// Don't hammer the bus; ~200 Hz max.
		delay(5);
	}

	if (samples == 0) {
		_accelBiasX = 0;
		_accelBiasY = 0;
		return false;
	}

	_accelBiasX = static_cast<int16_t>(sumX / static_cast<int64_t>(samples));
	_accelBiasY = static_cast<int16_t>(sumY / static_cast<int64_t>(samples));
	return true;
}

Gyro::AllRaw Gyro::readAll() {
	AllRaw out{};
	if (!_initialized) {
		return out;
	}

	uint8_t buf[14] = {0};
	if (!readBytes(REG_ACCEL_XOUT_H, buf, sizeof(buf))) {
		return out;
	}

	auto rd16 = [&](int idx) -> int16_t {
		return static_cast<int16_t>((static_cast<uint16_t>(buf[idx]) << 8) | static_cast<uint16_t>(buf[idx + 1]));
	};

	out.accel.x = rd16(0);
	out.accel.y = rd16(2);
	out.accel.z = rd16(4);
	out.temp.raw = rd16(6);
	out.gyro.x = rd16(8);
	out.gyro.y = rd16(10);
	out.gyro.z = rd16(12);

	// Apply calibration bias (defaults are 0 if not calibrated).
	out.accel.x = static_cast<int16_t>(out.accel.x - _accelBiasX);
	out.accel.y = static_cast<int16_t>(out.accel.y - _accelBiasY);
	out.gyro.x = static_cast<int16_t>(out.gyro.x - _gyroBiasX);
	out.gyro.y = static_cast<int16_t>(out.gyro.y - _gyroBiasY);
	out.gyro.z = static_cast<int16_t>(out.gyro.z - _gyroBiasZ);
	return out;
}

Gyro::AccelRaw Gyro::readAccel() {
	AccelRaw out{};
	if (!_initialized) {
		return out;
	}

	uint8_t buf[6] = {0};
	if (!readBytes(REG_ACCEL_XOUT_H, buf, sizeof(buf))) {
		return out;
	}

	out.x = static_cast<int16_t>((static_cast<uint16_t>(buf[0]) << 8) | static_cast<uint16_t>(buf[1]));
	out.y = static_cast<int16_t>((static_cast<uint16_t>(buf[2]) << 8) | static_cast<uint16_t>(buf[3]));
	out.z = static_cast<int16_t>((static_cast<uint16_t>(buf[4]) << 8) | static_cast<uint16_t>(buf[5]));

	// Apply calibration bias (defaults are 0 if not calibrated). Z intentionally untouched.
	out.x = static_cast<int16_t>(out.x - _accelBiasX);
	out.y = static_cast<int16_t>(out.y - _accelBiasY);
	return out;
}

Gyro::GyroRaw Gyro::readGyro() {
	GyroRaw out{};
	if (!_initialized) {
		return out;
	}

	uint8_t buf[6] = {0};
	if (!readBytes(REG_GYRO_XOUT_H, buf, sizeof(buf))) {
		return out;
	}

	out.x = static_cast<int16_t>((static_cast<uint16_t>(buf[0]) << 8) | static_cast<uint16_t>(buf[1]));
	out.y = static_cast<int16_t>((static_cast<uint16_t>(buf[2]) << 8) | static_cast<uint16_t>(buf[3]));
	out.z = static_cast<int16_t>((static_cast<uint16_t>(buf[4]) << 8) | static_cast<uint16_t>(buf[5]));

	// Apply calibration bias (defaults are 0 if not calibrated).
	out.x = static_cast<int16_t>(out.x - _gyroBiasX);
	out.y = static_cast<int16_t>(out.y - _gyroBiasY);
	out.z = static_cast<int16_t>(out.z - _gyroBiasZ);
	return out;
}

Gyro::TempRaw Gyro::readTemp() {
	TempRaw out{};
	if (!_initialized) {
		return out;
	}

	uint8_t buf[2] = {0};
	if (!readBytes(REG_TEMP_OUT_H, buf, sizeof(buf))) {
		return out;
	}

	out.raw = static_cast<int16_t>((static_cast<uint16_t>(buf[0]) << 8) | static_cast<uint16_t>(buf[1]));
	return out;
}

Gyro::AccelHR Gyro::accelHR(const AccelRaw& raw) {
	return accelHR(raw, AccelRange::g_16);
}

Gyro::AccelHR Gyro::accelHR(const AccelRaw& raw, AccelRange range) {
	// raw -> g -> m/s^2
	AccelHR out{};
	const float lsbPerG = accelLSBPerGForRange(range);
	out.x_mps2 = (static_cast<float>(raw.x) / lsbPerG) * kG;
	out.y_mps2 = (static_cast<float>(raw.y) / lsbPerG) * kG;
	out.z_mps2 = (static_cast<float>(raw.z) / lsbPerG) * kG;
	return out;
}

Gyro::GyroHR Gyro::gyroHR(const GyroRaw& raw) {
	GyroHR out{};
	const float lsbPerDps = gyroLSBPerDPSForRange(GyroRange::deg_per_sec250);
	out.x_dps = static_cast<float>(raw.x) / lsbPerDps;
	out.y_dps = static_cast<float>(raw.y) / lsbPerDps;
	out.z_dps = static_cast<float>(raw.z) / lsbPerDps;
	return out;
}

Gyro::GyroHR Gyro::gyroHR(const GyroRaw& raw, GyroRange range) {
	GyroHR out{};
	const float lsbPerDps = gyroLSBPerDPSForRange(range);
	out.x_dps = static_cast<float>(raw.x) / lsbPerDps;
	out.y_dps = static_cast<float>(raw.y) / lsbPerDps;
	out.z_dps = static_cast<float>(raw.z) / lsbPerDps;
	return out;
}

Gyro::TempHR Gyro::tempHR(const TempRaw& raw) {
	TempHR out{};
	out.c = (static_cast<float>(raw.raw) / 340.0f) + 36.53f;
	return out;
}

Gyro::AllHR Gyro::allHR(const AllRaw& raw) {
	AllHR out{};
	out.accel = accelHR(raw.accel);
	out.gyro = gyroHR(raw.gyro);
	out.temp = tempHR(raw.temp);
	return out;
}

Gyro::AllHR Gyro::allHR(const AllRaw& raw, GyroRange range) {
	AllHR out{};
	out.accel = accelHR(raw.accel);
	out.gyro = gyroHR(raw.gyro, range);
	out.temp = tempHR(raw.temp);
	return out;
}

void Gyro::printAccel(const AccelRaw& raw, Print& out) {
	out.printf("AccelRaw: x=%d y=%d z=%d\n", raw.x, raw.y, raw.z);
}

void Gyro::printGyro(const GyroRaw& raw, Print& out) {
	out.printf("GyroRaw:  x=%d y=%d z=%d\n", raw.x, raw.y, raw.z);
}

void Gyro::printTemp(const TempRaw& raw, Print& out) {
	out.printf("TempRaw:  raw=%d\n", raw.raw);
}

void Gyro::printAll(const AllRaw& raw, Print& out) {
	printAccel(raw.accel, out);
	printTemp(raw.temp, out);
	printGyro(raw.gyro, out);
}

void Gyro::printAccel(const AccelHR& hr, Print& out) {
	out.printf("Accel:   x=%7.3f y=%7.3f z=%7.3f m/s^2\n", hr.x_mps2, hr.y_mps2, hr.z_mps2);
}

void Gyro::printGyro(const GyroHR& hr, Print& out) {
	out.printf("Gyro:    x=%7.2f y=%7.2f z=%7.2f deg/s\n", hr.x_dps, hr.y_dps, hr.z_dps);
}

void Gyro::printTemp(const TempHR& hr, Print& out) {
	out.printf("Temp:    %6.2f C\n", hr.c);
}

void Gyro::printAll(const AllHR& hr, Print& out) {
	printAccel(hr.accel, out);
	printTemp(hr.temp, out);
	printGyro(hr.gyro, out);
}

bool Gyro::writeReg(uint8_t reg, uint8_t value) {
	_wire->beginTransmission(_address);
	_wire->write(reg);
	_wire->write(value);
	return _wire->endTransmission(true) == 0;
}

bool Gyro::readBytes(uint8_t startReg, uint8_t* buf, size_t len) {
	_wire->beginTransmission(_address);
	_wire->write(startReg);
	if (_wire->endTransmission(false) != 0) {
		return false;
	}

	const size_t got = _wire->requestFrom(static_cast<int>(_address), static_cast<int>(len), static_cast<int>(true));
	if (got != len) {
		// Drain whatever we got.
		for (size_t i = 0; i < got; i++) {
			(void)_wire->read();
		}
		return false;
	}

	for (size_t i = 0; i < len; i++) {
		buf[i] = static_cast<uint8_t>(_wire->read());
	}
	return true;
}


// #include <Arduino.h>
// #include <Gyro.h>

// static Gyro gyro;

// static constexpr uint32_t PRINT_INTERVAL_MS = 500;
// static uint32_t lastPrintMs = 0;

// void setup() {
// 	Serial.begin(115200);
// 	delay(250);
// 	Serial.println("\n\n=== Gyro (MPU6050) Test ===");

// 	// If you know your I2C pins, pass them here.
// 	// Otherwise, leave as defaults (depends on your board wiring).
// 	const int SDA_PIN = 8;
// 	const int SCL_PIN = 6;
// 	const bool ok = gyro.init(SDA_PIN, SCL_PIN, 0x68, 100000);
// 	Serial.printf("gyro.init: %s\n", ok ? "OK" : "FAIL");
//   gyro.calibrate(5000);
//   Serial.println("Gyro calibrated.");
// }

// void loop() {
// 	const uint32_t now = millis();
// 	if ((now - lastPrintMs) < PRINT_INTERVAL_MS) {
// 		return;
// 	}
// 	lastPrintMs = now;

// 	// Read individual domains (these do NOT call readAll()).
// 	const auto a = gyro.readAccel();
// 	const auto g = gyro.readGyro();
// 	const auto t = gyro.readTemp();

// 	Serial.println("\n-- Raw (single-domain reads) --");
// 	Gyro::printAccel(a);
// 	Gyro::printGyro(g);
// 	Gyro::printTemp(t);

// 	Serial.println("-- Human readable --");
// 	Gyro::printAccel(Gyro::accelHR(a));
// 	Gyro::printGyro(Gyro::gyroHR(g));
// 	Gyro::printTemp(Gyro::tempHR(t));

// 	// Optional: one-shot burst read for everything.
// 	Serial.println("-- Burst readAll() + HR --");
// 	const auto all = gyro.readAll();
// 	Gyro::printAll(all);
// 	Gyro::printAll(Gyro::allHR(all));
// }
