#include "GpsTrackingModule.h"
#include "gps/GPS.h"
#include "MeshService.h"
#include "MeshTypes.h"
#include "configuration.h"
#include "main.h"
#include "Throttle.h"
#include <Adafruit_LIS3DH.h>
#include <math.h>
#include <pb_encode.h>
#include "RTC.h"

GpsTrackingModule *gpsTrackingModule;

// --- Tune these thresholds ---
#define MOTION_THRESHOLD 1.2f       // Acceleration delta (m/s²) to count as motion
#define TRACKING_INTERVAL_MS 15000  // How often to send GPS when active
#define IDLE_TIMEOUT_MS 300000      // 5 minutes of no motion before turning off GPS
#define GPS_LOCK_TIMEOUT_MS 30000   // Max time to wait for GPS fix before sleeping again
// Polling intervals: adaptive for low power
#define POLL_ACTIVE_MS 200          // Poll accel every 200ms when moving
// -----------------------------

#define LIS3DH_ADDR 0x18

static Adafruit_LIS3DH lis;
static bool lisInitialized = false;
static bool gpsIsOn = false;

GpsTrackingModule::GpsTrackingModule()
    : SinglePortModule("gps_tracking", meshtastic_PortNum_UNKNOWN_APP)
    , concurrency::OSThread("GpsTracking")
{
    initLIS3DH();
    setIntervalFromNow(0);
}

void GpsTrackingModule::initLIS3DH()
{
    if (lisInitialized)
        return;

    if (lis.begin(LIS3DH_ADDR)) {
        lis.setRange(LIS3DH_RANGE_2_G);
        lis.setDataRate(LIS3DH_DATARATE_LOWPOWER_1K6HZ);
        lisInitialized = true;
        LOG_INFO("GpsTrackingModule: LIS3DH init ok (low-power 1.6kHz) on 0x%02X", LIS3DH_ADDR);
    } else {
        LOG_WARN("GpsTrackingModule: LIS3DH init failed on 0x%02X", LIS3DH_ADDR);
    }
}

void GpsTrackingModule::enableGps()
{
    if (!gpsIsOn) {
        config.position.gps_update_interval = 8; // < 10 to prevent gps go to sleep
        config.position.position_broadcast_secs = 300;
        gps->up();
        gpsIsOn = true;
        LOG_INFO("GpsTrackingModule: GPS powered ON");
    }
}

void GpsTrackingModule::disableGps()
{
    if (gpsIsOn) {
        config.position.gps_update_interval = 3 * 60 * 60; // 3hr
        config.position.position_broadcast_secs = 3 * 60 * 60; // 3hr
        gps->down();
        gpsIsOn = false;
        LOG_INFO("GpsTrackingModule: GPS powered OFF");
    }
}

bool GpsTrackingModule::isMotionDetected()
{
    lis.read();
    sensors_event_t event;
    lis.getEvent(&event);

    float magnitude = sqrt(event.acceleration.x * event.acceleration.x +
                           event.acceleration.y * event.acceleration.y +
                           event.acceleration.z * event.acceleration.z);
    float delta = fabs(magnitude - prevMagnitude);
    prevMagnitude = magnitude;

    static uint8_t motionStreak = 0;
    constexpr uint8_t REQUIRED_STREAK = 3;

    // LOG_INFO("GpsTrackingModule: delta=%.2f thresh=%.2f", delta, MOTION_THRESHOLD);
    if (delta > MOTION_THRESHOLD) {
        motionStreak++;
        if (motionStreak >= REQUIRED_STREAK) {
            motionStreak = REQUIRED_STREAK;
            LOG_INFO("GpsTrackingModule: delta=%.2f thresh=%.2f - motion detected!", delta, MOTION_THRESHOLD);
            return true;
        }
    } else {
        motionStreak = 0;
    }
    
    return false;
}

int32_t GpsTrackingModule::runOnce()
{
    // LOG_INFO("GpsTrackingModule: runOnce()");
    unsigned long now = millis();
    bool motion = isMotionDetected();

    if (motion) {
        lastMotionMs = now;

        if (!active) {
            active = true;
            lastLocationSendMs = now;
            enableGps();
            LOG_INFO("GpsTrackingModule: Motion detected - GPS Tracking On (motion)");
        }
    }

    if (active) {
        unsigned long idle = now - lastMotionMs;

        if (idle >= IDLE_TIMEOUT_MS) {
            active = false;
            disableGps();
            LOG_INFO("GpsTrackingModule: No motion for 5 min - GPS Tracking Off");
            return POLL_ACTIVE_MS;
        }

        if (Throttle::hasElapsed(lastLocationSendMs, TRACKING_INTERVAL_MS)) {
            lastLocationSendMs = now;
            sendGpsPayload(motion);
            if (motion) {
                LOG_INFO("GpsTrackingModule: GPS Tracking On (motion)");
            } else {
                LOG_INFO("GpsTrackingModule: GPS Tracking On (no-motion)");
            }
        }

        return POLL_ACTIVE_MS;
    }

    return POLL_ACTIVE_MS;
}

// uint32_t gpsLockedAtMs = 0;
int32_t lastGoodLatitudeI = 0;
int32_t lastGoodLongitudeI = 0;
int32_t lastGoodAltitude = 0;
uint32_t lastGoodTimestamp = 0;
bool hasGoodFix = false;
uint32_t lastPayloadSentMs = 0;

void GpsTrackingModule::sendGpsPayload(bool motionActive)
{
    if (!gps || !gpsStatus) {
        LOG_INFO("sendGpsPayload: !gps || !gpsStatus");
        return;
    }

    uint32_t now = millis();
    bool hasLock = gpsStatus->getHasLock();
    UltraCompactPayload payload = {};

    // --- 1. Coordinate & Navigation Logic ---
    if (hasLock) {
        double lat = gps->p.latitude_i / 1e7;
        double lon = gps->p.longitude_i / 1e7;
        
        payload.latitude_packed  = (uint32_t)((lat + 90.0) * (16777215.0 / 180.0));
        payload.longitude_packed = (uint32_t)((lon + 180.0) * (16777215.0 / 360.0));
        payload.altitude         = (gps->p.altitude > 0) ? (gps->p.altitude & 0xFFF) : 0;
        payload.course           = (uint32_t)(gps->p.ground_track / 5.625) & 0x3F;
        payload.num_sats         = (gps->p.sats_in_view > 15) ? 15 : gps->p.sats_in_view;

        // Use GPS epoch time if available; fallback to system RTC
        if (gps->p.timestamp > 0) {
            payload.short_timestamp = (uint16_t)(gps->p.timestamp & 0xFFFF);
        } else {
            uint32_t rtcSecs = getValidTime(RTCQuality::RTCQualityDevice);
            payload.short_timestamp = (rtcSecs > 0) ? (uint16_t)(rtcSecs & 0xFFFF) : 0;
        }

        // Cache coordinates for fallback when lock is lost
        lastGoodLatitudeI  = gps->p.latitude_i;
        lastGoodLongitudeI = gps->p.longitude_i;
        lastGoodAltitude   = gps->p.altitude;
        hasGoodFix         = true;
    } else {
        // Fallback to last known coordinates if valid fix existed previously
        if (hasGoodFix) {
            payload.latitude_packed  = (uint32_t)(((lastGoodLatitudeI / 1e7) + 90.0) * (16777215.0 / 180.0));
            payload.longitude_packed = (uint32_t)(((lastGoodLongitudeI / 1e7) + 180.0) * (16777215.0 / 360.0));
            payload.altitude         = (lastGoodAltitude > 0) ? (lastGoodAltitude & 0xFFF) : 0;
        } else {
            payload.latitude_packed  = 0;
            payload.longitude_packed = 0;
            payload.altitude         = 0;
        }

        // Retrieve system RTC time
        uint32_t epochSecs = getValidTime(RTCQuality::RTCQualityDevice);
        if (epochSecs > 0) {
            time_t rawTime = (time_t)epochSecs;
            struct tm *timeInfo = gmtime(&rawTime);
            if (timeInfo) {
                char timeString[9];
                strftime(timeString, sizeof(timeString), "%H:%M:%S", timeInfo);
                LOG_INFO("Formatted time: %s", timeString);
            }
            payload.short_timestamp = (uint16_t)(epochSecs & 0xFFFF);
        } else {
            LOG_INFO("Formatted time: [RTC Not Synced]");
            payload.short_timestamp = 0;
        }

        payload.course   = 0;
        payload.num_sats = 0;
    }

    // --- 2. Power & Battery Telemetry ---
    if (powerStatus && powerStatus->getHasBattery()) {
        float voltage = powerStatus->getBatteryVoltageMv() / 1000.0f;
        int batteryLevel = (int)((voltage - 3.2f) / 0.03f);
        payload.battery_packed = (batteryLevel < 0) ? 0 : (batteryLevel > 31) ? 31 : batteryLevel;
    } else {
        payload.battery_packed = 0;
    }

    // --- 3. System Flags & Charging State ---
    payload.fix_status     = hasLock ? 1 : 0;
    payload.vibration_flag = motionActive ? 1 : 0;

    if (powerStatus) {
        if (powerStatus->getBatteryChargePercent() > 99) {
            payload.charging_state = 0x02; // Battery Full
        } else if (powerStatus->getIsCharging()) {
            payload.charging_state = 0x01; // Charging
        } else {
            payload.charging_state = 0x00; // Discharging / Battery mode
        }

        LOG_INFO("GpsTracking: battery=%d mV, pct=%d%%, usb=%d, charging=%d, state=%d",
                  powerStatus->getBatteryVoltageMv(),
                  powerStatus->getBatteryChargePercent(),
                  powerStatus->getHasUSB(),
                  powerStatus->getIsCharging(),
                  payload.charging_state);
    } else {
        payload.charging_state = 0x00;
    }

    // --- 4. Node Hardware Identification ---
    memcpy(payload.device_id, &owner.macaddr[2], 4);

    // --- 5. Parity / Checksum Calculation ---
    uint8_t checksum = 0;
    const uint8_t *raw = (const uint8_t *)&payload;
    for (uint8_t i = 0; i < offsetof(UltraCompactPayload, parity); i++) {
        checksum ^= raw[i];
    }
    payload.parity = checksum;

    // --- 6. Base64 Encoding (17 payload bytes -> 18 byte block -> 24 Base64 chars) ---
    uint8_t raw18[18] = {0};
    memcpy(raw18, raw, sizeof(payload)); // Copy 17 bytes; 18th byte remains 0 padding for 3-byte Base64 alignment
    
    static const char b64[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    char b64buf[25];
    for (int i = 0; i < 6; i++) {
        uint32_t val = ((uint32_t)raw18[i * 3] << 16) | ((uint32_t)raw18[i * 3 + 1] << 8) | raw18[i * 3 + 2];
        b64buf[i * 4]     = b64[(val >> 18) & 0x3F];
        b64buf[i * 4 + 1] = b64[(val >> 12) & 0x3F];
        b64buf[i * 4 + 2] = b64[(val >> 6) & 0x3F];
        b64buf[i * 4 + 3] = b64[val & 0x3F];
    }
    b64buf[24] = '\0';
    LOG_INFO("GpsTracking: sent gps_payload %s (%s)", b64buf, hasLock ? "fix" : "no-fix");

    // --- 7. Dispatch via Meshtastic Mesh ---
    meshtastic_MeshPacket *p = allocDataPacket();
    p->to = NODENUM_BROADCAST;
    p->decoded.portnum = meshtastic_PortNum_TEXT_MESSAGE_APP;
    p->decoded.payload.size = strlen(b64buf);
    memcpy(p->decoded.payload.bytes, b64buf, strlen(b64buf));

    service->sendToMesh(p);
    lastPayloadSentMs = now;
}