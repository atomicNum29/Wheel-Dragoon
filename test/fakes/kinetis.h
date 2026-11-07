#pragma once

#include <stdint.h>
extern uint32_t SCB_AIRCR;

#if FIRMWARE_TIMING_DIAGNOSTICS
extern uint32_t ARM_DEMCR, ARM_DWT_CTRL, ARM_DWT_CYCCNT;
constexpr uint32_t ARM_DEMCR_TRCENA = 1u << 24;
constexpr uint32_t ARM_DWT_CTRL_CYCCNTENA = 1u;
constexpr uint32_t F_CPU = 72000000u;
#endif
