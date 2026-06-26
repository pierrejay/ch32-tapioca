// led_blinker.cpp - cooperative LED command runner.
#include "led_blinker.hpp"
#include "time.hpp"

LedBlinker::LedBlinker(GPIO_TypeDef* port,
                       uint32_t pin,
                       uint32_t rccPeriph,
                       bool isDirect,
                       uint32_t minTimeOffMs)
    : port_(port),
      pin_(pin),
      rccPeriph_(rccPeriph),
      direct_(isDirect),
      minTimeOff_(minTimeOffMs)
{
}

void LedBlinker::init()
{
    RCC_APB2PeriphClockCmd(rccPeriph_, ENABLE);

    GPIO_InitTypeDef gpio = {0};
    gpio.GPIO_Pin = pin_;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(port_, &gpio);

    initialized_ = true;
    setLed(false);
}

bool LedBlinker::on()
{
    return blink(UINT32_MAX);
}

bool LedBlinker::off()
{
    return blink(0);
}

bool LedBlinker::blink(uint32_t timeOnMs)
{
    return blink(timeOnMs, 0);
}

bool LedBlinker::blink(uint32_t timeOnMs, uint32_t timeOffMs)
{
    BlinkCommand cmd = {timeOnMs, timeOffMs};
    return queue_.push(cmd);
}

void LedBlinker::tick()
{
    if (!initialized_)
    {
        return;
    }

    uint32_t now = Time::millis();

    if (phase_ == Phase::Idle)
    {
        BlinkCommand cmd = {};
        if (queue_.pop(&cmd))
        {
            start(cmd, now);
        }
        return;
    }

    if (!expired(now, deadline_))
    {
        return;
    }

    if (phase_ == Phase::On)
    {
        setLed(false);
        uint32_t timeOff = current_.timeOff ? current_.timeOff : minTimeOff_;
        deadline_ = now + timeOff;
        phase_ = Phase::Off;
        return;
    }

    phase_ = Phase::Idle;
} // LedBlinker::tick()

bool LedBlinker::expired(uint32_t now, uint32_t deadline)
{
    return (int32_t)(now - deadline) >= 0;
}

void LedBlinker::setLed(bool state)
{
    bool pinHigh = direct_ ? state : !state;
    if (pinHigh)
    {
        GPIO_SetBits(port_, pin_);
    }
    else
    {
        GPIO_ResetBits(port_, pin_);
    }
}

void LedBlinker::start(const BlinkCommand &cmd, uint32_t now)
{
    current_ = cmd;

    if (cmd.timeOn == UINT32_MAX)
    {
        setLed(true);
        phase_ = Phase::Idle;
        return;
    }

    if (cmd.timeOn == 0)
    {
        setLed(false);
        phase_ = Phase::Idle;
        return;
    }

    setLed(true);
    deadline_ = now + cmd.timeOn;
    phase_ = Phase::On;
}
