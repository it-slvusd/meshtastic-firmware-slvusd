#pragma once
#include "ProtobufModule.h"
#include "mesh/generated/meshtastic/portnums.pb.h"
#include "concurrency/OSThread.h"
#include "../mesh/SinglePortModule.h"

#pragma pack(push, 1)
/**
 * @brief 17-Byte Ultra-Compact Payload Structure
 * Struct layout is explicitly packed to prevent compiler padding bytes.
 * Total bitfields: 95 bits (12 bytes, 7 bits padded to 12 bytes) + 4 bytes ID + 1 byte parity = 17 bytes total.
 */
struct UltraCompactPayload {
    // --- COORDINATES (48 bits / 6 bytes) ---
    // Rescaled to 24-bit integers (~1.1 meter accuracy).
    // Formula: (Latitude + 90.0) * (16777215.0 / 180.0)
    uint32_t latitude_packed  : 24; 
    // Formula: (Longitude + 180.0) * (16777215.0 / 360.0)
    uint32_t longitude_packed : 24; 

    // --- TIME (16 bits / 2 bytes) ---
    // Truncated lower 16 bits of Unix epoch seconds. Rollover every ~18.2 hours.
    uint32_t short_timestamp  : 16; 

    // --- NAVIGATION (18 bits) ---
    uint32_t altitude         : 12; // 0 to 4095 meters absolute altitude
    uint32_t course           : 6;  // 0 to 63 (~5.625° steps. Formula: heading / 5.625)

    // --- TELEMETRY (5 bits) ---
    // Battery: Range 3.2V to 4.13V in 0.03V increments. Formula: (Voltage - 3.2) / 0.03
    uint32_t battery_packed   : 5;  

    // --- STATUS & FLAGS (4 bits) ---
    uint32_t num_sats         : 4;  // 0 to 15 satellites (15 = 15 or more)

    // --- SYSTEM BITS (3 bits) ---
    uint32_t fix_status       : 1;  // 0 = No Fix, 1 = Fix
    uint32_t vibration_flag   : 1;  // 0 = Stationary, 1 = Motion Active
    uint32_t charging_state   : 2;  // 00 = Discharging/Battery, 01 = Charging, 10 = Full, 11 = Reserved

    // --- DEVICE IDENTIFIER (4 bytes) ---
    uint8_t device_id[4];           // Last 4 bytes of Bluetooth MAC address

    // --- INTEGRITY CHECK (1 byte) ---
    uint8_t parity;                 // Cumulative XOR checksum of preceding 16 bytes
} __attribute__((packed));
#pragma pack(pop)

class GpsTrackingModule : public SinglePortModule, private concurrency::OSThread
{
public:
    GpsTrackingModule();

protected:
    int32_t runOnce() override;

private:
    void initLIS3DH();
    void wakeUp();
    void goSleep();
    void enableGps();
    void disableGps();
    void enableLora();
    void disableLora();
    bool isMotionDetected();
    void sendGpsPayload(bool motionActive);

    unsigned long lastLocationSendMs = 0;
    unsigned long lastMotionMs = 0;
    unsigned long lastLoraSleepMs = 0;
    unsigned long lastBatteryCheckMs = 0;

    float gravityX = 0.0f;
    float gravityY = 0.0f;
    float gravityZ = 0.0f;
    bool isGravityInitialized = false;

    bool active = false;

    uint8_t motionStreak = 0;
    uint8_t lowBattStreak = 0;
};

extern GpsTrackingModule *gpsTrackingModule;
