// ch32_sdk.hpp - single include point for the WCH CH32X035 SDK from C++.
//
// WCH's <ch32x035_usb.h> has a header bug. Under `#ifdef __cplusplus` it opens
// `extern "C" {` TWICE (lines 17 and 21) but closes it only ONCE (line 517).
// We include the SDK headers here and add the one compensating brace.
#pragma once

extern "C" {
#include "debug.h"
#include <ch32x035_usb.h>
}   // extern "C" above
}   // extern "C" left open by <ch32x035_usb.h>
