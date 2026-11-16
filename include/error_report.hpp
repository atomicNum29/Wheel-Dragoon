#pragma once

// A detected error is reported this many times. Detecting it again reloads
// the remaining count; only fully written, still-current 0x87 snapshots consume it.
constexpr int ERROR_REPORT_REPEAT_COUNT = 10;
