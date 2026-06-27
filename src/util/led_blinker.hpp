// led_blinker.hpp - non-blocking GPIO LED blinker for the WCH noneos SDK.
//
// Queue blink/on/off commands with on(), off(), blink(), then call tick() from
// the main loop. No RTOS task, no blocking delay.
#pragma once

#include <stdint.h>
#include "ch32_sdk.hpp"
#include "ring.hpp"

#define LED_BLINK_QUEUE_SIZE 10
#define LED_BLINK_MIN_TIME_OFF_MS 20

class LedBlinker
{
public:
    LedBlinker(GPIO_TypeDef* port,
               uint32_t pin,
               uint32_t rccPeriph,
               bool isDirect = true,
               uint32_t minTimeOffMs = LED_BLINK_MIN_TIME_OFF_MS);

    void init();
    void begin() { init(); }
    void tick();

    bool on();
    bool off();
    bool blink(uint32_t timeOnMs);
    bool blink(uint32_t timeOnMs, uint32_t timeOffMs);

    bool idle() const { return phase_ == Phase::Idle && queue_.empty(); }

private:
    struct BlinkCommand
    {
        uint32_t timeOn;
        uint32_t timeOff;
    };

    enum class Phase : uint8_t
    {
        Idle,
        On,
        Off
    };

    static bool expired(uint32_t now, uint32_t deadline);

    void setLed(bool state);
    void start(const BlinkCommand& cmd, uint32_t now);

    GPIO_TypeDef* port_;
    uint32_t pin_;
    uint32_t rccPeriph_;
    bool direct_;
    bool initialized_ = false;
    uint32_t minTimeOff_;

    Ring<LED_BLINK_QUEUE_SIZE, BlinkCommand> queue_;

    BlinkCommand current_ = {};
    Phase phase_ = Phase::Idle;
    uint32_t deadline_ = 0;
}; // class LedBlinker
