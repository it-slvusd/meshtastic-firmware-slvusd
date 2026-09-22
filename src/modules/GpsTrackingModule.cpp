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

/*
    Running 40mA
    Idle    >1mA
    if Vbat < 3300, reboot -> stucked in boot loop until reach 3500
*/

GpsTrackingModule *gpsTrackingModule;

#define REQUIRED_STREAK_LOWBAT      5

#define TRACKING_INTERVAL_MS    15000  // How often to send GPS when active
#define IDLE_TIMEOUT_MS         300000      // 5 minutes of no motion before turning off GPS

#define LORA_SLEEP_RECHECK      30 * 1000

#define GPS_UPDATE_SECS         8
#define GPS_UPDATE_SECS_IDLE    3 * 60 * 60 // 3hr
#define GPS_BROADCAST_SECS      3 * 60 * 60 // 3hr
// -----------------------------

#define LIS3DH_ADDR 0x18

static Adafruit_LIS3DH lis;
static bool lisInitialized = false;
static bool gpsIsOn = false;
static bool sleeping = false;

GpsTrackingModule::GpsTrackingModule()
    : SinglePortModule("gps_tracking", meshtastic_PortNum_UNKNOWN_APP)
    , concurrency::OSThread("GpsTracking")
{
    initLIS3DH();
    config.power.is_power_saving = false;
    config.bluetooth.enabled = false;
    
    config.device.role = meshtastic_Config_DeviceConfig_Role_TRACKER;
    config.device.rebroadcast_mode = meshtastic_Config_DeviceConfig_RebroadcastMode_ALL;

    config.position.gps_update_interval = GPS_UPDATE_SECS;
    config.position.position_broadcast_secs = GPS_BROADCAST_SECS;
    gpsIsOn = true;

    active = true;
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

void GpsTrackingModule::enableGpsAndLora()
{
    sleeping = false;
    config.device.led_heartbeat_disabled = false;

    if (!gpsIsOn) {
        config.power.is_power_saving = false;
        config.position.gps_update_interval = GPS_UPDATE_SECS; 
        config.position.position_broadcast_secs = GPS_BROADCAST_SECS;
        gps->up();
        gpsIsOn = true;
        LOG_INFO("GpsTrackingModule: GPS powered ONNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNN");
    }

    RadioInterface *rif = router ? router->getRadioIface() : nullptr;
    if (rif) {
        config.device.role = meshtastic_Config_DeviceConfig_Role_TRACKER;
        config.device.rebroadcast_mode = meshtastic_Config_DeviceConfig_RebroadcastMode_ALL;
        rif->sleeping = false;
        rif->reconfigure();
        // rif->init();
    }
}

void GpsTrackingModule::disableGpsAndLora()
{
    sleeping = true;
    config.device.led_heartbeat_disabled = true;

    if (gpsIsOn) {
        config.position.gps_update_interval = GPS_UPDATE_SECS_IDLE;
        config.position.position_broadcast_secs = GPS_BROADCAST_SECS;
        gps->down();
        gpsIsOn = false;
        LOG_INFO("GpsTrackingModule: GPS powered OFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF");
    }

    RadioInterface *rif = router ? router->getRadioIface() : nullptr;
    if (rif) {
        uint8_t retries = 0;
        const uint8_t maxRetries = 20; // Prevent infinite loop (3-second max timeout)

        config.power.is_power_saving = true;
        config.device.role = meshtastic_Config_DeviceConfig_Role_CLIENT_MUTE;
        config.device.rebroadcast_mode = meshtastic_Config_DeviceConfig_RebroadcastMode_NONE;

        while (!rif->canSleep(true /* deepSleep */) && retries < maxRetries) {
            LOG_DEBUG("GpsTrackingModule: Waiting for RadioInterface to allow sleep (%d/%d)", retries + 1, maxRetries);
            delay(300);
            retries++;
        }

        rif->sleeping = true; // hacking flag to skip SX126xInterface<T>::resetAGC() which will wake up rif every 60 sec.
        if (rif->canSleep(true)) {
            rif->sleep();
            LOG_INFO("GpsTrackingModule: RadioInterface Sleep!");
        } else {
            LOG_WARN("GpsTrackingModule: RadioInterface busy, sleep skipped?");
            // i don't care, sleep now!
            rif->sleep();
        }
    }
}

// --- Tune these parameters ---
#define POLL_ACTIVE_MS              100        // Poll accel every 100ms
#define MOTION_THRESHOLD            0.6f       // Dynamic acceleration delta (m/s²)
#define REQUIRED_STREAK             5

// Filter coefficient (0.0 to 1.0). Higher values adapt slower to tilt changes.
// At 100ms polling, 0.90f creates a ~1 second LPF smoothing window for gravity.
#define ALPHA_GRAVITY               0.90f      

bool GpsTrackingModule::isMotionDetected()
{
    lis.read();
    sensors_event_t event;
    lis.getEvent(&event);

    float ax = event.acceleration.x;
    float ay = event.acceleration.y;
    float az = event.acceleration.z;

    // First run initialization: set static baseline to initial reading
    if (!isGravityInitialized) {
        gravityX = ax;
        gravityY = ay;
        gravityZ = az;
        isGravityInitialized = true;
        return false;
    }

    // 1. Low-Pass Filter: Update static gravity & tilt vector per axis
    gravityX = ALPHA_GRAVITY * gravityX + (1.0f - ALPHA_GRAVITY) * ax;
    gravityY = ALPHA_GRAVITY * gravityY + (1.0f - ALPHA_GRAVITY) * ay;
    gravityZ = ALPHA_GRAVITY * gravityZ + (1.0f - ALPHA_GRAVITY) * az;

    // 2. High-Pass Filter: Subtract static gravity vector to isolate dynamic motion
    float dynX = ax - gravityX;
    float dynY = ay - gravityY;
    float dynZ = az - gravityZ;

    // 3. Compute 3D magnitude of dynamic motion only
    float dynamicAcc = sqrt(dynX * dynX + dynY * dynY + dynZ * dynZ);

    // 4. Leaky bucket streak counter
    if (dynamicAcc > MOTION_THRESHOLD) {
        motionStreak++;
        if (motionStreak >= REQUIRED_STREAK) {
            motionStreak = REQUIRED_STREAK; // Cap streak counter
            // LOG_INFO("Motion detected! dynamicAcc=%.2f m/s²", dynamicAcc);
            return true;
        }
    } else {
        if (motionStreak > 0) {
            motionStreak--;
        }
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
            enableGpsAndLora();
            LOG_INFO("GpsTrackingModule: Motion detected - GPS Tracking On (motion)");
        }
    }

    if (active) {
        unsigned long idle = now - lastMotionMs;
        unsigned long idle_threshold = (powerStatus->getHasUSB())? 35*1000:IDLE_TIMEOUT_MS; // usb = test mode, idle time = 30 sec

        if (idle >= idle_threshold) {
            active = false;
            lastLoraSleepMs = now;
            disableGpsAndLora();
            // LOG_INFO("GpsTrackingModule: No motion for 5 min - GPS Tracking Off");
            return POLL_ACTIVE_MS;
        }

        if (Throttle::hasElapsed(lastLocationSendMs, TRACKING_INTERVAL_MS)) {
            lastLocationSendMs = now;
            sendGpsPayload(motion);

            LOG_INFO("GpsTrackingModule: runOnce - idle for %ld sec", idle/1000);
        }

        return POLL_ACTIVE_MS;
    }

    // Lora wake up by something else, put it too sleep again. every 300 sec
    if (sleeping && Throttle::hasElapsed(lastLoraSleepMs, LORA_SLEEP_RECHECK)) {
        lastLoraSleepMs = now;
        disableGpsAndLora();
    }

    // Reboot to boot loop until power OK
    int BattMv = powerStatus->getBatteryVoltageMv();
    if (BattMv < SAFE_VDD_VOLTAGE_THRESHOLD_MV) {
        lowBattStreak++;

        if (lowBattStreak >= REQUIRED_STREAK_LOWBAT ) {
            LOG_INFO("GpsTracking: Battery %d (too low)", BattMv);
            if (!powerStatus->getHasUSB()) {

                // send 1 packet before die
                sendGpsPayload(false);
                delay(5000);

                digitalWrite(PIN_3V3_EN, LOW);
                rebootAtMsec = millis() + 5000;
                return disable();
            }
        }
    } else {
        lowBattStreak = 0;
    }

    return POLL_ACTIVE_MS;
}

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
                // LOG_INFO("Formatted time: %s", timeString);
            }
            payload.short_timestamp = (uint16_t)(epochSecs & 0xFFFF);
        } else {
            // LOG_INFO("Formatted time: [RTC Not Synced]");
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