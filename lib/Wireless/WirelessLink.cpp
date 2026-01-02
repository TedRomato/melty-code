#include "WirelessLink.h"

#include <cstring>

WirelessLink* WirelessLink::_instance = nullptr;

namespace {
constexpr size_t kEspNowMaxPayload = 250;
constexpr uint32_t kDefaultPingIntervalMs = 200;

static void printMac(const char* label, const uint8_t mac[6]) {
	if (!label || !mac) {
		return;
	}
	Serial.printf("%s%02X:%02X:%02X:%02X:%02X:%02X\n", label, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}
}

WirelessLink::WirelessLink() {
	// Intentionally minimal. Real setup occurs in initCommunication().
}

bool WirelessLink::initCommunication(const uint8_t peerMac[6], int initialChannel, wifi_power_t txPowerDbm) {
	if (!peerMac) {
		return false;
	}

	std::memcpy(_peerMac, peerMac, 6);
	_currentChannel = initialChannel;
	_txPowerDbm = static_cast<int8_t>(txPowerDbm);

	Serial.println("\n→ WirelessLink: init WiFi (STA + ESP-NOW)");
	WiFi.mode(WIFI_STA);
	WiFi.disconnect(false, true);
	WiFi.setSleep(false);

	uint8_t selfMac[6]{};
	if (esp_read_mac(selfMac, ESP_MAC_WIFI_STA) == ESP_OK) {
		printMac("WirelessLink: self STA MAC: ", selfMac);
	}
	printMac("WirelessLink: peer MAC: ", _peerMac);
	Serial.printf("WirelessLink: channel=%d txPowerEnum=%d\n", _currentChannel, (int)txPowerDbm);

	esp_err_t err = esp_wifi_start();
	Serial.printf("WirelessLink: esp_wifi_start=%d (%s)\n", (int)err, esp_err_to_name(err));

	esp_wifi_set_promiscuous(true);
	err = esp_wifi_set_channel(static_cast<uint8_t>(_currentChannel), WIFI_SECOND_CHAN_NONE);
	esp_wifi_set_promiscuous(false);
	Serial.printf("WirelessLink: esp_wifi_set_channel=%d (%s)\n", (int)err, esp_err_to_name(err));

	err = esp_wifi_set_max_tx_power(_txPowerDbm);
	Serial.printf("WirelessLink: esp_wifi_set_max_tx_power=%d (%s)\n", (int)err, esp_err_to_name(err));

	uint8_t primary;
	wifi_second_chan_t second;
	if (esp_wifi_get_channel(&primary, &second) == ESP_OK) {
		Serial.printf("WirelessLink: home channel: %u\n", primary);
	}

	if (!initEspNow()) {
		Serial.println("WirelessLink: failed to init ESP-NOW");
		return false;
	}

	// Add peer
	esp_now_peer_info_t peerInfo{};
	std::memcpy(peerInfo.peer_addr, _peerMac, 6);
	peerInfo.channel = 0; // use current channel
	peerInfo.ifidx = WIFI_IF_STA;
	peerInfo.encrypt = false;

	// If it already exists, remove and re-add.
	esp_now_del_peer(_peerMac);
	if (esp_now_add_peer(&peerInfo) != ESP_OK) {
		Serial.println("WirelessLink: failed to add peer");
		return false;
	}

	_initialized = true;
	_connected = false;
	_connectNotified = false;
	_lastPingMs = 0;
	_pingIntervalMs = kDefaultPingIntervalMs;
	return true;
}

bool WirelessLink::sendData(const void* data, size_t len) {
	return sendPacket(MsgType::USER_DATA, data, len);
}

void WirelessLink::setOnDataReceived(DataCallback cb) {
	_onData = std::move(cb);
}

void WirelessLink::setOnConnect(ConnectCallback cb) {
	_onConnect = std::move(cb);
}

bool WirelessLink::isConnected() const {
	return _connected;
}

void WirelessLink::update() {
	if (!_initialized) {
		return;
	}

	processPings(millis());

	// Drain event queue
	while (_eventTail != _eventHead) {
		const Event ev = _eventQueue[_eventTail];
		_eventTail = static_cast<uint8_t>((_eventTail + 1) % MAX_EVENTS);

		if (ev.type == EventType::RECV_PACKET) {
			handleRecv(ev.mac, ev.data, ev.len);
		} else if (ev.type == EventType::SEND_STATUS) {
			handleSendStatus(ev.mac, ev.sendStatus);
		}
	}

	if (_connected && !_connectNotified) {
		_connectNotified = true;
		if (_onConnect) {
			_onConnect();
		}
	}
}

// -------------------------
// ESP-NOW callbacks (static)
// -------------------------

void WirelessLink::_onEspNowRecv(const uint8_t* mac, const uint8_t* data, int len) {
	if (!_instance || !mac || !data || len <= 0) {
		return;
	}

	const uint8_t nextHead = static_cast<uint8_t>((_instance->_eventHead + 1) % MAX_EVENTS);
	if (nextHead == _instance->_eventTail) {
		// queue full; drop
		return;
	}

	Event& ev = _instance->_eventQueue[_instance->_eventHead];
	ev.type = EventType::RECV_PACKET;
	std::memcpy(ev.mac, mac, 6);

	const int copyLen = (len > static_cast<int>(sizeof(ev.data))) ? static_cast<int>(sizeof(ev.data)) : len;
	std::memcpy(ev.data, data, copyLen);
	ev.len = static_cast<uint16_t>(copyLen);

	_instance->_eventHead = nextHead;
}

void WirelessLink::_onEspNowSent(const uint8_t* mac, esp_now_send_status_t status) {
	if (!_instance || !mac) {
		return;
	}

	const uint8_t nextHead = static_cast<uint8_t>((_instance->_eventHead + 1) % MAX_EVENTS);
	if (nextHead == _instance->_eventTail) {
		return;
	}

	Event& ev = _instance->_eventQueue[_instance->_eventHead];
	ev.type = EventType::SEND_STATUS;
	std::memcpy(ev.mac, mac, 6);
	ev.sendStatus = status;
	ev.len = 0;
	_instance->_eventHead = nextHead;
}

// -------------------------
// Internal processing
// -------------------------

void WirelessLink::handleRecv(const uint8_t* mac, const uint8_t* data, int len) {
	if (!mac || !data || len < static_cast<int>(sizeof(PacketHeader))) {
		return;
	}

	// Only accept packets from our configured peer.
	// (We still enqueue in ISR for speed; filter here.)
	if (std::memcmp(mac, _peerMac, 6) != 0) {
		return;
	}

	const auto* header = reinterpret_cast<const PacketHeader*>(data);
	const size_t payloadLen = header->payloadLen;
	const size_t totalNeeded = sizeof(PacketHeader) + payloadLen;
	if (totalNeeded > static_cast<size_t>(len)) {
		return;
	}

	const uint8_t* payload = data + sizeof(PacketHeader);
	const MsgType type = static_cast<MsgType>(header->type);

	switch (type) {
		case MsgType::USER_DATA:
			if (_onData) {
				_onData(payload, payloadLen);
			}
			break;

		case MsgType::PING:
			// Peer is online; respond.
			(void)sendPacket(MsgType::PONG, nullptr, 0);
			break;

		case MsgType::PONG:
			// We got a response to our ping.
			_connected = true;
			break;

		default:
			break;
	}
}

void WirelessLink::handleSendStatus(const uint8_t* /*mac*/, esp_now_send_status_t /*status*/) {
	// Placeholder for future ACK/retry logic. Intentionally no-op.
}

bool WirelessLink::sendPacket(MsgType type, const void* data, size_t len) {
	if (!_initialized) {
		return false;
	}
	if (!data && len != 0) {
		return false;
	}

	const size_t maxPayload = kEspNowMaxPayload - sizeof(PacketHeader);
	if (len > maxPayload) {
		Serial.println("WirelessLink: sendPacket payload too large");
		return false;
	}

	uint8_t buffer[kEspNowMaxPayload]{};
	auto* header = reinterpret_cast<PacketHeader*>(buffer);
	header->type = static_cast<uint8_t>(type);
	header->reserved = 0;
	header->payloadLen = static_cast<uint16_t>(len);
	header->packetNum = ++_seqCounter;

	if (len > 0) {
		std::memcpy(buffer + sizeof(PacketHeader), data, len);
	}

	const size_t totalLen = sizeof(PacketHeader) + len;
	const esp_err_t result = esp_now_send(_peerMac, buffer, totalLen);
	return result == ESP_OK;
}

void WirelessLink::processPings(uint32_t nowMs) {
	if (_connected) {
		return;
	}

	if (_lastPingMs == 0 || (nowMs - _lastPingMs) >= _pingIntervalMs) {
		_lastPingMs = nowMs;
		(void)sendPacket(MsgType::PING, nullptr, 0);
	}
}

bool WirelessLink::setChannelInternal(int ch) {
	if (ch < 1 || ch > 13) {
		return false;
	}

	esp_wifi_set_promiscuous(true);
	const esp_err_t err = esp_wifi_set_channel(static_cast<uint8_t>(ch), WIFI_SECOND_CHAN_NONE);
	esp_wifi_set_promiscuous(false);
	if (err != ESP_OK) {
		return false;
	}
	_currentChannel = ch;

	if (esp_now_is_peer_exist(_peerMac)) {
		esp_now_del_peer(_peerMac);
	}

	esp_now_peer_info_t peerInfo{};
	std::memcpy(peerInfo.peer_addr, _peerMac, 6);
	peerInfo.channel = 0; // use current channel
	peerInfo.ifidx = WIFI_IF_STA;
	peerInfo.encrypt = false;
	(void)esp_now_add_peer(&peerInfo);
	return true;
}

bool WirelessLink::initEspNow() {
	_instance = this;

	// Best-effort: if already initialized elsewhere, reset cleanly.
	(void)esp_now_deinit();

	const esp_err_t initErr = esp_now_init();
	Serial.printf("WirelessLink: esp_now_init=%d (%s)\n", (int)initErr, esp_err_to_name(initErr));
	if (initErr != ESP_OK) {
		return false;
	}

	if (esp_now_register_recv_cb(&WirelessLink::_onEspNowRecv) != ESP_OK) {
		return false;
	}
	if (esp_now_register_send_cb(&WirelessLink::_onEspNowSent) != ESP_OK) {
		return false;
	}

	// Reset event queue indices
	_eventHead = 0;
	_eventTail = 0;
	return true;
}



/*
WirelessLink Symmetric Test (no-ACK)

Both devices run the same logic:
 - Send the same frame type at a fixed interval
 - Verify received frames using a deterministic payload pattern

Only difference between controller_main.cpp and melty_main.cpp:
 - The peerMac[] value
*/

/*


#include <Arduino.h>
#include <cstring>

#include "WirelessLink.h"



#ifndef ESPNOW_CHANNEL
#define ESPNOW_CHANNEL 6
#endif

// TODO: Replace with the other ESP32's STA MAC.
static uint8_t peerMac[6] = {0x1C, 0xDB, 0xD4, 0xC4, 0x67, 0x40};

static WirelessLink radio;

static constexpr bool kInitiator = false;
static uint32_t connectedAtMs = 0;

struct __attribute__((packed)) HiMsg {
  uint8_t kind;   // always 1
  uint8_t word;   // 0=hi, 1=hey
  uint32_t counter;
};

static constexpr uint8_t WORD_HI = 0;
static constexpr uint8_t WORD_HEY = 1;

static const char* wordToStr(uint8_t w) {
  return (w == WORD_HEY) ? "hey" : "hi";
}

static uint32_t hiCounter = 0;
static bool replyPending = false;
static uint32_t replyDueMs = 0;

static void sendHi() {
  HiMsg msg{};
  msg.kind = 1;
  msg.word = WORD_HEY;
  msg.counter = ++hiCounter;
  (void)radio.sendData(&msg, sizeof(msg));
  Serial.printf("TX: %s #%lu\n", wordToStr(msg.word), (unsigned long)msg.counter);
}

void setup() {
  Serial.begin(115200);
  delay(250);
  Serial.println("\n\n===== WirelessLink Hi/Hi Demo =====");

  radio.setOnConnect([]() {
    Serial.println("CONNECTED (ping/pong handshake OK)");
    connectedAtMs = millis();
  });

  radio.setOnDataReceived([](const uint8_t* data, size_t len) {
    if (!data || len < sizeof(HiMsg)) {
      return;
    }

    HiMsg msg{};
    std::memcpy(&msg, data, sizeof(msg));
    if (msg.kind != 1) {
      return;
    }

    Serial.printf("RX: %s #%lu\n", wordToStr(msg.word), (unsigned long)msg.counter);
    replyPending = true;
    replyDueMs = millis() + 1000;
  });

  const bool ok = radio.initCommunication(peerMac, ESPNOW_CHANNEL, WIFI_POWER_13dBm);
  Serial.printf("initCommunication: %s\n", ok ? "OK" : "FAIL");
}

void loop() {
  radio.update();

  if (!radio.isConnected()) return;

  // Wait a bit after connect before starting app messages.
  if (connectedAtMs != 0 && static_cast<int32_t>(millis() - connectedAtMs) < 3000) {
    return;
  }

  const uint32_t now = millis();
  if (replyPending && static_cast<int32_t>(now - replyDueMs) >= 0) {
    replyPending = false;
    sendHi();
  }
}


*/