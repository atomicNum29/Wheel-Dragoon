#pragma once

// A detected error is reported this many times. Detecting it again reloads
// the remaining count; only transmitted canonical 0x82 packets consume it.
constexpr int ERROR_REPORT_REPEAT_COUNT = 10;
