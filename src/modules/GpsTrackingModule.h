#pragma once

#ifndef _GPS_TRACKING_MODULE_H_
#define _GPS_TRACKING_MODULE_H_

#include "configuration.h"

#if !MESHTASTIC_EXCLUDE_GPSTRACKING && !MESHTASTIC_EXCLUDE_GPS

#include "../concurrency/OSThread.h"

class GpsTrackingModule : public concurrency::OSThread
{
  public:
    GpsTrackingModule();
    ~GpsTrackingModule();

    int32_t runOnce() override;
};

extern GpsTrackingModule *gpsTrackingModule;

#endif
#endif