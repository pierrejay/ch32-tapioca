// usb_descriptors.hpp - USB-CDC (ACM) descriptor tables for the CH32X035.
//
// Same wire bytes as the original WCH example, just renamed and grouped.
// Endpoint layout (full speed):
//   EP0          control, 64 B
//   EP1 IN  0x81 interrupt, CDC notifications (enumerated, unused here)
//   EP2 OUT 0x02 bulk, host -> device data
//   EP3 IN  0x83 bulk, device -> host data
#pragma once

#include <stdint.h>

namespace usb_desc
{

// Device identity
constexpr uint16_t kVid     = 0x1A86;   // WCH
constexpr uint16_t kPid     = 0xFE0C;
constexpr uint8_t  kEp0Size = 64;       // bMaxPacketSize0
constexpr uint16_t kPacket  = 64;       // bulk/interrupt max packet (FS)

// String descriptor indices
enum StringId : uint8_t
{
    kStrLang    = 0x00,
    kStrManu    = 0x01,
    kStrProd    = 0x02,
    kStrSerial  = 0x03,
};

// Device descriptor
constexpr uint8_t kDevice[] =
{
    0x12,                                   // bLength
    0x01,                                   // bDescriptorType: Device
    0x10, 0x01,                             // bcdUSB 1.10
    0x02,                                   // bDeviceClass: CDC
    0x00,                                   // bDeviceSubClass
    0x00,                                   // bDeviceProtocol
    kEp0Size,                               // bMaxPacketSize0
    (uint8_t)kVid, (uint8_t)(kVid >> 8),    // idVendor
    (uint8_t)kPid, (uint8_t)(kPid >> 8),    // idProduct
    0x01, 0x00,                             // bcdDevice 0.01
    kStrManu,                               // iManufacturer
    kStrProd,                               // iProduct
    0x00,                                   // iSerialNumber (none advertised in device desc)
    0x01,                                   // bNumConfigurations
};

// Configuration descriptor (CDC ACM: comm interface + data interface)
constexpr uint8_t kConfig[] =
{
    // Configuration
    0x09, 0x02, 0x43, 0x00, 0x02, 0x01, 0x00, 0x80, 0x32,

    // Interface 0 - CDC communication
    0x09, 0x04, 0x00, 0x00, 0x01, 0x02, 0x02, 0x01, 0x00,

    // CDC functional descriptors
    0x05, 0x24, 0x00, 0x10, 0x01,           // Header
    0x05, 0x24, 0x01, 0x00, 0x01,           // Call Management
    0x04, 0x24, 0x02, 0x02,                 // ACM
    0x05, 0x24, 0x06, 0x00, 0x01,           // Union

    // EP1 IN - interrupt (notifications)
    0x07, 0x05, 0x81, 0x03, (uint8_t)kPacket, (uint8_t)(kPacket >> 8), 0x01,

    // Interface 1 - CDC data
    0x09, 0x04, 0x01, 0x00, 0x02, 0x0A, 0x00, 0x00, 0x00,

    // EP2 OUT - bulk (host -> device)
    0x07, 0x05, 0x02, 0x02, (uint8_t)kPacket, (uint8_t)(kPacket >> 8), 0x00,

    // EP3 IN - bulk (device -> host)
    0x07, 0x05, 0x83, 0x02, (uint8_t)kPacket, (uint8_t)(kPacket >> 8), 0x00,
};

// String 0 - supported languages (English US)
constexpr uint8_t kLang[] = { 0x04, 0x03, 0x09, 0x04 };

// String 1 - manufacturer "wch.cn"
constexpr uint8_t kManufacturer[] =
{
    0x0E, 0x03, 'w', 0, 'c', 0, 'h', 0, '.', 0, 'c', 0, 'n', 0
};

// String 2 - product "USB Serial"
constexpr uint8_t kProduct[] =
{
    0x16, 0x03, 'U', 0, 'S', 0, 'B', 0, ' ', 0, 'S', 0, 'e', 0,
                'r', 0, 'i', 0, 'a', 0, 'l', 0
};

// String 3 - serial number "0123456789"
constexpr uint8_t kSerial[] =
{
    0x16, 0x03, '0', 0, '1', 0, '2', 0, '3', 0, '4', 0, '5', 0,
                '6', 0, '7', 0, '8', 0, '9', 0
};

// Descriptor lengths are encoded in the tables themselves:
//   device : byte[0]
//   config : little-endian word at byte[2..3]
//   strings: byte[0]
inline uint16_t deviceLen() { return kDevice[0]; }
inline uint16_t configLen() { return (uint16_t)kConfig[2] | ((uint16_t)kConfig[3] << 8); }

} // namespace usb_desc
