#include <Arduino.h>

#include "WirelessLink.h"

#include "Controller.h"
#include "Interface.h"

// ============================
// Controller (handheld) main
// ============================

#ifndef ESPNOW_CHANNEL
#define ESPNOW_CHANNEL 6
#endif

// Robot (melty) ESP32 STA MAC: 10:00:3B:BC:73:D8
static uint8_t peerMac[6] = {0x10, 0x00, 0x3B, 0xBC, 0x73, 0xD8};

static WirelessLink radio;

namespace {
class RadioAdapter final : public IRadio {
public:
	explicit RadioAdapter(WirelessLink& link) : _link(link) {}
	bool isConnected() const override { return _link.isConnected(); }
	bool isConnectionLost() const override { return _link.isConnectionLost(); }
	bool send(const void* data, size_t len) override { return _link.sendData(data, len); }

private:
	WirelessLink& _link;
};
}

static RadioAdapter radioAdapter(radio);
static Controller controller(radioAdapter);

// Interface config - hardware enabled via USE_HARDWARE build flag
static Interface::Config uiConfig;

static void initUiConfig() {
#ifdef USE_HARDWARE
	// OLED + joystick + encoders - adjust pins for your wiring
	uiConfig.oledAddr = 0x3C;
	uiConfig.joystickXPin = 3;
	uiConfig.joystickYPin = 4;
	uiConfig.joystickBtnPin = 7;
	uiConfig.enc1PinA = 0;
	uiConfig.enc1PinB = 8;
	uiConfig.i2cSda = 5;
	uiConfig.i2cScl = 6;
	uiConfig.enc2PinA = 21;
	uiConfig.enc2PinB = 20;
	uiConfig.enc3PinA = 1;
	uiConfig.enc3PinB = 2;
	uiConfig.saveBtnPin = 10;  // Save button pin
	uiConfig.buzzerPin = 9;  // Buzzer pin
	uiConfig.enableHardware = true;
#else
	uiConfig.i2cSda = 5;
	uiConfig.i2cScl = 6;
	uiConfig.saveBtnPin = -1;
	uiConfig.enableHardware = false;
#endif
}

static Interface* ui = nullptr;

void setup() {
	Serial.begin(115200);
	delay(3000);
	
#ifdef USE_HARDWARE
	Serial.println("\n\n=== Melty Controller (Hardware) ===");
#else
	Serial.println("\n\n=== Melty Controller (Keyboard) ===");
#endif

	// Load saved settings from NVS before UI init
	controller.loadSettings();

	initUiConfig();
	ui = new Interface(controller, uiConfig);
	ui->begin();
	
	// Configure heartbeat and connection loss detection on radio
	radio.setConnectionLostTimeout(5000);  // Connection lost if no data for 5 seconds
	radio.runHeartbeat(2500);              // Send ping every 2.5 seconds
	
	radio.setOnConnectionLost([]() {
		Serial.println("Wireless: DISCONNECTED (no response from melty)");
	});

	radio.setOnConnect([]() {
		Serial.println("Wireless: CONNECTED");
		controller.onConnect();
	});

	radio.setOnDataReceived([](const uint8_t* data, size_t len) {
		controller.handleRx(data, len);
	});

	const bool ok = radio.initCommunication(peerMac, ESPNOW_CHANNEL, WIFI_POWER_13dBm);
	Serial.printf("Wireless init: %s\n", ok ? "OK" : "FAIL");
}

void loop() {
	radio.update();
	controller.update();
	if (ui) ui->update();
}