// spi_gen.hpp - SPI1 loopback generator (PA5=SCK, PA7=MOSI).
//
// BENCH/DEMO ONLY. Drives a known byte stream into the PIOC capture pins
// (jumper PA5->PC18, PA7->PC19) so the datapath can be validated without a real
// clocked bus. A real passive tap NEVER configures these pins (see ClockedSniffer),
// so the generator can't drive a live bus.
//
// Stateless: a tiny namespace of inline helpers, safe to include anywhere.
#pragma once

#include "ch32_sdk.hpp"

namespace SpiGen {

inline void init(uint16_t prescaler, uint16_t cpha)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_SPI1, ENABLE);
    GPIO_InitTypeDef g = {0};
    g.GPIO_Pin   = GPIO_Pin_5 | GPIO_Pin_7;
    g.GPIO_Mode  = GPIO_Mode_AF_PP;
    g.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &g);

    SPI_Cmd(SPI1, DISABLE);
    SPI_InitTypeDef si = {0};
    si.SPI_Direction         = SPI_Direction_2Lines_FullDuplex;
    si.SPI_Mode              = SPI_Mode_Master;
    si.SPI_DataSize          = SPI_DataSize_8b;
    si.SPI_CPOL              = SPI_CPOL_Low;
    si.SPI_CPHA              = cpha;
    si.SPI_NSS               = SPI_NSS_Soft;
    si.SPI_BaudRatePrescaler = prescaler;
    si.SPI_FirstBit          = SPI_FirstBit_MSB;
    SPI_Init(SPI1, &si);
    SPI_Cmd(SPI1, ENABLE);
}

inline void tx(uint8_t b)
{
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET) { }
    SPI_I2S_SendData(SPI1, b);
}

inline void waitDone()
{
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == SET) { }
}

} // namespace SpiGen
