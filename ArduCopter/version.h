#pragma once

#include "ap_version.h"

#define THISFIRMWARE "APM:Copter V3.5.2-flow-alt_mode"
#define FIRMWARE_VERSION 3,5,2,FIRMWARE_VERSION_TYPE_OFFICIAL

#ifndef GIT_VERSION
#define FIRMWARE_STRING THISFIRMWARE
#else
#define FIRMWARE_STRING THISFIRMWARE " (" GIT_VERSION ")"
#endif
