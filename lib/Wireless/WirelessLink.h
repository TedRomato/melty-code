// WirelessLink.h
// Tiny helper library for robust ESP-NOW communication between two peers
// (e.g. melty brain robot + remote controller).
//
// Responsibilities:
//  - Initialize WiFi (STA) + ESP-NOW on a given channel and TX power
//  - Manage a single peer (MAC address)
//  - Handle heartbeats (send + timeout detection)
//  - Handle internal control messages (heartbeat, link stats, etc.)
//  - Provide simple sendData() for user payloads
//  - Track link quality over a sliding time window
//  - Optionally scan for best WiFi channel (startup / manual)
//
// Things intentionally NOT handled here:
//  - Your motor logic, ESC arming, etc. (that’s the app’s job)
//  - Application-level message formats (except a small internal header)
//  - Automatic channel switching during normal operation (you trigger it)

#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <functional>

// ==========================
// CONFIG + TYPES
// ==========================

class WirelessLink {
public:
    // Type of payload carried by a packet
    enum class MsgType : uint8_t {
        USER_DATA = 0x01, // your payloads

        // Internal link management messages (handled inside the library)
        PING = 0xF0,
        PONG = 0xF1
    };

    // Wire header prepended to every WirelessLink packet
    struct PacketHeader {
        uint8_t  type;       // MsgType
        uint8_t  reserved;   // for alignment
        uint16_t payloadLen; // bytes following the header
        uint16_t packetNum;  // incrementing sequence number for debugging
    };

    // User callbacks
    using DataCallback             = std::function<void(const uint8_t* data, size_t len)>;
    using ConnectCallback          = std::function<void()>;

public:
    WirelessLink();

    // --------------------------------------------------------
    // 1. Basic init + peer setup
    // --------------------------------------------------------

    // Initialize WiFi in STA mode, set channel + TX power,
    // initialize ESP-NOW, register callbacks, add a single peer.
    //
    //  - peerMac: 6-byte STA MAC of the remote device
    //  - initialChannel: Wi-Fi channel (1-13) to start on
    //  - txPowerDbm: transmit power in dBm (see wifi_power_t)
    //
    // Returns: true on success, false on failure (check Serial logs for reason).
    bool initCommunication(const uint8_t peerMac[6],
                           int initialChannel = 6,
                           wifi_power_t txPowerDbm = WIFI_POWER_13dBm);

    // --------------------------------------------------------
    // 2. User data send / receive
    // --------------------------------------------------------

    // Send user payload (e.g. control frame).
    bool sendData(const void* data, size_t len);

    // Register callback for USER_DATA messages.
    // Only application-level packets should reach this callback.
    // Internal messages (HEARTBEAT / LINK_QUERY / LINK_REPORT / CHANNEL_*) are
    // consumed inside the library and never shown here.
    // Callback is invoked from update(), not from interrupt.
    void setOnDataReceived(DataCallback cb);

    // Called once when the peer has been detected online via internal ping/pong.
    // Callback is invoked from update() (main context), never from ESP-NOW ISR.
    void setOnConnect(ConnectCallback cb);

    // True once we've received a PONG from the peer.
    bool isConnected() const;

    // Call this regularly from your main loop().
    //
    // Responsibilities:
    //  - Rotate link-stats buckets based on time
    //  - Process all queued ESP-NOW events (receive + send status)
    //    and invoke user callbacks from the MAIN CONTEXT
    //
    // IMPORTANT:
    //  - All user callbacks (onDataReceived)
    //    are ONLY called from
    //    inside update() — never from the ESP-NOW interrupt context.
    //  - This allows you to do heavier work (Serial prints, math, etc.)
    //    inside callbacks without blocking the radio.
    void update();

private:
    // Internal state ---------------------------------------------------

    uint8_t _peerMac[6]{};

    bool _initialized      = false;
    int  _currentChannel   = 1;
    int8_t _txPowerDbm = 52;

    // Callbacks
    DataCallback             _onData       = nullptr;
    ConnectCallback          _onConnect    = nullptr;

    // Connection state
    bool _connected = false;
    bool _connectNotified = false;
    uint32_t _lastPingMs = 0;
    uint32_t _pingIntervalMs = 200;

    // Sequence number for user data (for debugging / future use)
    uint16_t _seqCounter = 0;

    // Singleton pointer for static ESP-NOW callbacks
    static WirelessLink* _instance;

    // Types of internal events produced by ESP-NOW callbacks
    enum class EventType : uint8_t {
        NONE,
        RECV_PACKET,
        SEND_STATUS
    };

    struct Event {
        EventType type = EventType::NONE;
        uint8_t   mac[6];           // sender/peer MAC
        esp_now_send_status_t sendStatus;  // for SEND_STATUS
        // Simple buffer for received data; you can tune sizes.
        uint8_t  data[250];         // ESP-NOW max payload is 250 bytes
        uint16_t len = 0;
    };

    static constexpr uint8_t MAX_EVENTS = 16;   // small ring buffer
    Event   _eventQueue[MAX_EVENTS];
    uint8_t _eventHead = 0; // write index (callback side)
    uint8_t _eventTail = 0; // read index (update() side);

    // Internal methods -------------------------------------------------

    static void _onEspNowRecv(const uint8_t *mac, const uint8_t *data, int len);
    static void _onEspNowSent(const uint8_t *mac, esp_now_send_status_t status);


    void handleRecv(const uint8_t *mac, const uint8_t *data, int len);
    void handleSendStatus(const uint8_t *mac, esp_now_send_status_t status);

    bool sendPacket(MsgType type, const void* data, size_t len);
    void processPings(uint32_t nowMs);

    bool setChannelInternal(int ch);
    bool initEspNow();
};

// ==========================
// TODO (future enhancements)
// ==========================
// - Acknowledgments and retries
// - Heartbeat keep-alive + timeout callback
// - Link quality / packet counters over a sliding window
// - WiFi channel scanning (noise scores) + optional channel switching

