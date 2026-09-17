#include "GpsTrackingModule.h"

#if !MESHTASTIC_EXCLUDE_GPSTRACKING && !MESHTASTIC_EXCLUDE_GPS

GpsTrackingModule *gpsTrackingModule = nullptr;

GpsTrackingModule::GpsTrackingModule() : OSThread("GpsTracking")
{
    setIntervalFromNow(0);
    LOG_INFO("GpsTrackingModule::GpsTrackingModule() ======================================================= ");
}

GpsTrackingModule::~GpsTrackingModule() {}

int32_t GpsTrackingModule::runOnce()
{
    LOG_INFO("GpsTrackingModule::runOnce() ======================================================= ");

    return 200;
}

#endif