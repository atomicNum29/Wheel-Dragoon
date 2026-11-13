#pragma once

// PlatformIO environments are the only feature selection point.
#ifndef FIRMWARE_CAN_DEBUG
#define FIRMWARE_CAN_DEBUG 0
#endif
#ifndef FIRMWARE_TIMING_DIAGNOSTICS
#define FIRMWARE_TIMING_DIAGNOSTICS 0
#endif
#if (FIRMWARE_CAN_DEBUG != 0 && FIRMWARE_CAN_DEBUG != 1) || \
    (FIRMWARE_TIMING_DIAGNOSTICS != 0 && FIRMWARE_TIMING_DIAGNOSTICS != 1)
#error "Firmware features must be 0 or 1"
#endif
