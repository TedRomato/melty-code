#include <Arduino.h>
#include <Gyro.h>

static Gyro gyro;

static constexpr uint32_t PRINT_INTERVAL_MS = 500;
static uint32_t lastPrintMs = 0;

void setup() {
	Serial.begin(115200);
	delay(250);
	Serial.println("\n\n=== IMU Accel (MPU6050) Test ===");

	// If you know your I2C pins, pass them here.
	// Otherwise, leave as defaults (depends on your board wiring).
	const int SDA_PIN = 8;
	const int SCL_PIN = 6;
	const bool ok = gyro.init(SDA_PIN, SCL_PIN, 0x68, 100000);
	Serial.printf("gyro.init: %s\n", ok ? "OK" : "FAIL");
	gyro.calibrateAccel(5000);
	Serial.println("Accel calibrated.");
}

void loop() {
	const uint32_t now = millis();
	if ((now - lastPrintMs) < PRINT_INTERVAL_MS) {
		return;
	}
	lastPrintMs = now;

	// Read individual domains (these do NOT call readAll()).
	const auto a = gyro.readAccel();
	const auto t = gyro.readTemp();

	Serial.println("\n-- Raw (single-domain reads) --");
	Gyro::printAccel(a);
	Gyro::printTemp(t);

	Serial.println("-- Human readable --");
	Gyro::printAccel(Gyro::accelHR(a));
	Gyro::printTemp(Gyro::tempHR(t));
}
