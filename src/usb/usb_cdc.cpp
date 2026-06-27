// usb_cdc.cpp - implementation of the self-contained USB-CDC device.
//
// The control-transfer logic mirrors WCH's reference USBFS_IRQHandler, but the
// data path is decoupled: bulk OUT data lands in rx_, bulk IN data is drained
// from tx_ in tick(). No UART, no DMA aliasing into foreign buffers.
#include "usb_cdc.hpp"
#include "usb_descriptors.hpp"

extern "C" {
#include <string.h>
}

// ------------------------------------------------------------------
// Local register/endpoint constants (were scattered in WCH headers).
// ------------------------------------------------------------------
#define UEP_IN              0x80
#define UEP_OUT             0x00
#define UEP0                0x00
#define UEP1                0x01
#define UEP2                0x02
#define UEP3                0x03

// AFIO USB PHY control bits
#define USB_IOEN            0x00000080
#define USB_PHY_V33         0x00000040
#define UDP_PUE_MASK        0x0000000C
#define UDP_PUE_10K         0x00000008
#define UDP_PUE_1K5         0x0000000C
#define UDM_PUE_MASK        0x00000003

static constexpr uint16_t kEp0Size = usb_desc::kEp0Size;

UsbCdc* UsbCdc::self_ = nullptr;

// ------------------------------------------------------------------
// Setup / init
// ------------------------------------------------------------------
void UsbCdc::endpointInit()
{
    USBFSD->UEP4_1_MOD = USBFS_UEP1_TX_EN;
    USBFSD->UEP2_3_MOD = USBFS_UEP2_RX_EN | USBFS_UEP3_TX_EN;

    USBFSD->UEP0_DMA = (uint32_t)ep0_;
    USBFSD->UEP1_DMA = (uint32_t)ep1_;
    USBFSD->UEP2_DMA = (uint32_t)ep2_;
    USBFSD->UEP3_DMA = (uint32_t)ep3_;

    USBFSD->UEP0_CTRL_H = USBFS_UEP_R_RES_ACK | USBFS_UEP_T_RES_NAK;
    USBFSD->UEP2_CTRL_H = USBFS_UEP_R_RES_ACK;     // ready to receive bulk OUT

    USBFSD->UEP1_TX_LEN = 0;
    USBFSD->UEP3_TX_LEN = 0;
    USBFSD->UEP1_CTRL_H = USBFS_UEP_T_RES_NAK;
    USBFSD->UEP3_CTRL_H = USBFS_UEP_T_RES_NAK;

    ep3Busy_    = false;
    ep2Nak_     = false;
    partialPending_ = false;
}

void UsbCdc::init()
{
    self_ = this;
    decodeLineCoding();

    // USB clocks
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_USBFS, ENABLE);

    // USB DP/DM GPIO (PC16/PC17)
    GPIO_InitTypeDef gpio = {0};
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    gpio.GPIO_Pin = GPIO_Pin_16;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOC, &gpio);
    gpio.GPIO_Pin = GPIO_Pin_17;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOC, &gpio);

    // PHY pull-ups depend on the supply rail
    if (PWR_VDD_SupplyVoltage() == PWR_VDD_5V)
    {
        AFIO->CTLR = (AFIO->CTLR & ~(UDP_PUE_MASK | UDM_PUE_MASK | USB_PHY_V33)) | UDP_PUE_10K | USB_IOEN;
    }
    else
    {
        AFIO->CTLR = (AFIO->CTLR & ~(UDP_PUE_MASK | UDM_PUE_MASK)) | USB_PHY_V33 | UDP_PUE_1K5 | USB_IOEN;
    }

    USBFSD->BASE_CTRL = 0x00;
    endpointInit();
    USBFSD->DEV_ADDR  = 0x00;
    USBFSD->BASE_CTRL = USBFS_UC_DEV_PU_EN | USBFS_UC_INT_BUSY | USBFS_UC_DMA_EN;
    USBFSD->INT_FG    = 0xFF;
    USBFSD->UDEV_CTRL = USBFS_UD_PD_DIS | USBFS_UD_PORT_EN;
    USBFSD->INT_EN    = USBFS_UIE_SUSPEND | USBFS_UIE_BUS_RST | USBFS_UIE_TRANSFER;
    // USB polled from tick() in the main loop, NOT via ISR. The WCH-Interrupt-fast
    // attribute disables nesting (MIPS=0 for the whole ISR), so a 30-50us USB ISR
    // blocks TIM3 (prio 0, PIOC drain) -> PIOC ring overflow at 1 Mbit/s. Polling
    // USB from the main loop (every ~9us between TIM3 fires) makes TIM3 the only
    // ISR, never blocked. USB FS tolerates this (SETUP response window is ~2ms).
    // The peripheral still sets INT_FG (INT_EN is on); we just don't let the CPU
    // take the interrupt. 
    NVIC_DisableIRQ(USBFS_IRQn);
} // UsbCdc::init()

// ------------------------------------------------------------------
// Line coding
// ------------------------------------------------------------------
void UsbCdc::decodeLineCoding()
{
    lc_.baud     = (uint32_t)lcWire_[0]
                 | ((uint32_t)lcWire_[1] << 8)
                 | ((uint32_t)lcWire_[2] << 16)
                 | ((uint32_t)lcWire_[3] << 24);
    lc_.stopBits = lcWire_[4];
    lc_.parity   = lcWire_[5];
    lc_.dataBits = lcWire_[6];
}

// ------------------------------------------------------------------
// Bulk IN: load ep3_ (already filled by caller) and ACK
// ------------------------------------------------------------------
void UsbCdc::armEp3(uint16_t len)
{
    USBFSD->UEP3_TX_LEN = len;
    USBFSD->UEP3_CTRL_H = (USBFSD->UEP3_CTRL_H & ~USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_ACK;
    ep3Busy_ = true;
}

// ------------------------------------------------------------------
// Main-loop step
// ------------------------------------------------------------------
void UsbCdc::tick(uint32_t nowMs)
{
    // Poll the USB interrupt flags (ISR disabled — see init()). The main loop
    // calls tick() every ~9us (between TIM3 fires), fast enough for USB FS. 
    if (USBFSD->INT_FG & (USBFS_UIF_TRANSFER | USBFS_UIF_BUS_RST | USBFS_UIF_SUSPEND))
        onIrq();

    // NOTE: the NVIC_Disable/EnableIRQ(USBFS_IRQn) pairs that used to wrap the EP
    // accesses below were REMOVED. They were leftovers from an IRQ-driven design: USB
    // is polled here (init() disables USBFS_IRQn on purpose - a non-nesting
    // WCH-Interrupt-fast USB ISR would block TIM3/PIOC drain). But NVIC_EnableIRQ
    // RE-ENABLED the USB ISR every tick, so it fired on every transaction and preempted
    // the main loop (~1ms/packet -> loop crawled to ~800Hz, throughput capped ~28KB/s).
    // With the IRQ left off, USB is purely polled and the loop runs at its real rate. 

    // Re-open the OUT endpoint once the RX ring has room for a full packet.
    if (ep2Nak_ && rx_.free() >= usb_desc::kPacket)
    {
        USBFSD->UEP2_CTRL_H = (USBFSD->UEP2_CTRL_H & ~USBFS_UEP_R_RES_MASK) | USBFS_UEP_R_RES_ACK;
        ep2Nak_ = false;
    }

    // Coalesce the continuous capture stream: the fast path only arms full 64-byte
    // packets. A short packet is permitted solely after a bounded hold, otherwise
    // a low-rate bus (or a final log fragment) could wait forever. 
    if (!ep3Busy_)
    {
        uint32_t n = tx_.size();
        if (n >= usb_desc::kPacket)
        {
            partialPending_ = false;
            armEp3FromTx(false);
        }
        else if (n)
        {
            if (!partialPending_)
            {
                partialPending_ = true;
                partialSinceMs_ = nowMs;
            }
            if ((uint32_t)(nowMs - partialSinceMs_) >= kPartialFlushMs)
            {
                partialPending_ = false;
                armEp3FromTx(true);
            }
        }
        else
        {
            partialPending_ = false;
        }
    }
} // UsbCdc::tick()

// Pop one packet from tx_ into ep3_. The pump is deliberately full-packet-only;
// tick() is the single place allowed to flush a residual fragment after the timeout.
// No ZLP: EP3 carries a continuous, 0xFF-delimited capture stream. 
bool UsbCdc::armEp3FromTx(bool allowShort)
{
    uint32_t n = tx_.size();
    if (!n) return false;
    if (n < usb_desc::kPacket && !allowShort) return false;
    if (n > usb_desc::kPacket) n = usb_desc::kPacket;
    tx_.pop(ep3_, n);
    txBytes_ += n;                           // real bytes shipped (not pktHz*64)
    if (n == usb_desc::kPacket) ++fullPkts_; else ++shortPkts_;
    armEp3((uint16_t)n);
    return true;
}

// Lean bulk-IN pump - see header. Fast path: an EP3 IN just completed -> un-freeze and
// re-arm right here, without waiting for the next tick(). 
void UsbCdc::pump()
{
    if (!(USBFSD->INT_FG & USBFS_UIF_TRANSFER)) return;     // nothing completed since last check

    uint8_t intst = USBFSD->INT_ST;
    if ((intst & (USBFS_UIS_TOKEN_MASK | USBFS_UIS_ENDP_MASK)) != (USBFS_UIS_TOKEN_IN | UEP3))
    {
        // Rare event whose LAST token (INT_ST) is not EP3-IN: any EP0 SETUP (enumeration,
        // SET_LINE_CODING), EP2 OUT, etc. -> full handler. INT_ST shows only the last token,
        // so in theory an EP3-IN completion masked by a later non-IN event in the same
        // UIF_TRANSFER window could be missed. Why it's benign:
        //  - INT_BUSY=1 auto-pauses the SIE until UIF_TRANSFER is cleared, so completions
        //    serialize (one pending at a time) - the masking window is essentially nil.
        //  - Even if one were missed, ep3Busy_ just stays set and the NEXT tick() re-arms EP3
        //    (~180us later): a single late packet, NOT a permanent stall.
        //  - These events only occur at connect / line-coding change, not during streaming.
        // REVISIT if host->device data on EP2 becomes frequent. Worth a one-time stress test:
        // open the CDC port + change baud/line-state mid-capture and confirm EP3 keeps flowing. 
        onIrq();
        return;
    }

    // EP3 IN completion (same as the onIrq UEP3 case), then clear the flag to release the
    // INT_BUSY auto-NAK, then re-arm immediately. 
    USBFSD->UEP3_CTRL_H ^= USBFS_UEP_T_TOG;
    USBFSD->UEP3_CTRL_H = (USBFSD->UEP3_CTRL_H & ~USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_NAK;
    ep3Busy_ = false;
    txPkts_++;
    USBFSD->INT_FG = USBFS_UIF_TRANSFER;                    // un-freeze the endpoint
    if (armEp3FromTx(false)) partialPending_ = false;
}

// ------------------------------------------------------------------
// SETUP packet
// ------------------------------------------------------------------
void UsbCdc::handleSetup()
{
    PUSB_SETUP_REQ req = (PUSB_SETUP_REQ)ep0_;
    uint16_t len = 0;
    uint8_t  errflag = 0;

    USBFSD->UEP0_CTRL_H = USBFS_UEP_T_TOG | USBFS_UEP_T_RES_NAK | USBFS_UEP_R_TOG | USBFS_UEP_R_RES_NAK;

    descr_ = nullptr;   // fresh transaction: no descriptor staged yet
    setupReqType_ = req->bRequestType;
    setupReqCode_ = req->bRequest;
    setupLen_     = req->wLength;
    setupValue_   = req->wValue;
    setupIndex_   = req->wIndex;

    if ((setupReqType_ & USB_REQ_TYP_MASK) != USB_REQ_TYP_STANDARD)
    {
        // Class / vendor request
        if (setupReqType_ & USB_REQ_TYP_CLASS)
        {
            switch (setupReqCode_)
            {
                case CDC_GET_LINE_CODING:
                    descr_ = lcWire_;
                    len = 7;
                    break;
                case CDC_SET_LINE_CODING:    // data arrives in the OUT stage
                case CDC_SET_LINE_CTLSTE:    // DTR/RTS - ignored
                case CDC_SEND_BREAK:         // ignored
                    break;
                default:
                    errflag = 0xFF;
                    break;
            }
        }
        else
        {
            errflag = 0xFF;
        }

        len = (setupLen_ >= kEp0Size) ? kEp0Size : setupLen_;
        if (descr_) memcpy(ep0_, descr_, len);
        if (descr_) descr_ += len;
    }
    else
    {
        // Standard request
        switch (setupReqCode_)
        {
            case USB_GET_DESCRIPTOR:
                switch ((uint8_t)(setupValue_ >> 8))
                {
                    case USB_DESCR_TYP_DEVICE:
                        descr_ = usb_desc::kDevice;
                        len = usb_desc::deviceLen();
                        break;
                    case USB_DESCR_TYP_CONFIG:
                        descr_ = usb_desc::kConfig;
                        len = usb_desc::configLen();
                        break;
                    case USB_DESCR_TYP_STRING:
                        switch ((uint8_t)(setupValue_ & 0xFF))
                        {
                            case usb_desc::kStrLang:
                                descr_ = usb_desc::kLang;        len = usb_desc::kLang[0];        break;
                            case usb_desc::kStrManu:
                                descr_ = usb_desc::kManufacturer; len = usb_desc::kManufacturer[0]; break;
                            case usb_desc::kStrProd:
                                descr_ = usb_desc::kProduct;     len = usb_desc::kProduct[0];     break;
                            case usb_desc::kStrSerial:
                                descr_ = usb_desc::kSerial;      len = usb_desc::kSerial[0];      break;
                            default:
                                errflag = 0xFF;
                                break;
                        }
                        break;
                    default:
                        errflag = 0xFF;
                        break;
                }
                if (setupLen_ > len) setupLen_ = len;
                len = (setupLen_ >= kEp0Size) ? kEp0Size : setupLen_;
                if (descr_) memcpy(ep0_, descr_, len);
                if (descr_) descr_ += len;
                break;

            case USB_SET_ADDRESS:
                devAddr_ = (uint8_t)(setupValue_ & 0xFF);
                break;

            case USB_GET_CONFIGURATION:
                ep0_[0] = devConfig_;
                if (setupLen_ > 1) setupLen_ = 1;
                break;

            case USB_SET_CONFIGURATION:
                devConfig_   = (uint8_t)(setupValue_ & 0xFF);
                enumerated_  = true;
                break;

            case USB_CLEAR_FEATURE:
                if ((setupReqType_ & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_DEVICE)
                {
                    if ((uint8_t)(setupValue_ & 0xFF) == USB_REQ_FEAT_REMOTE_WAKEUP)
                        sleepStatus_ &= ~0x01;
                }
                else if ((setupReqType_ & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_ENDP)
                {
                    if ((uint8_t)(setupValue_ & 0xFF) == USB_REQ_FEAT_ENDP_HALT)
                    {
                        switch ((uint8_t)(setupIndex_ & 0xFF))
                        {
                            case (UEP_IN | UEP1):  USBFSD->UEP1_CTRL_H = USBFS_UEP_T_RES_NAK; break;
                            case (UEP_OUT | UEP2): USBFSD->UEP2_CTRL_H = USBFS_UEP_R_RES_ACK; break;
                            case (UEP_IN | UEP3):  USBFSD->UEP3_CTRL_H = USBFS_UEP_T_RES_NAK; break;
                            default: errflag = 0xFF; break;
                        }
                    }
                    else errflag = 0xFF;
                }
                else errflag = 0xFF;
                break;

            case USB_SET_FEATURE:
                if ((setupReqType_ & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_DEVICE)
                {
                    if ((uint8_t)(setupValue_ & 0xFF) == USB_REQ_FEAT_REMOTE_WAKEUP)
                    {
                        if (usb_desc::kConfig[7] & 0x20) sleepStatus_ |= 0x01;
                        else errflag = 0xFF;
                    }
                    else errflag = 0xFF;
                }
                else if ((setupReqType_ & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_ENDP)
                {
                    if ((uint8_t)(setupValue_ & 0xFF) == USB_REQ_FEAT_ENDP_HALT)
                    {
                        switch ((uint8_t)(setupIndex_ & 0xFF))
                        {
                            case (UEP_IN | UEP1):
                                USBFSD->UEP1_CTRL_H = (USBFSD->UEP1_CTRL_H & ~USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_STALL; break;
                            case (UEP_OUT | UEP2):
                                USBFSD->UEP2_CTRL_H = (USBFSD->UEP2_CTRL_H & ~USBFS_UEP_R_RES_MASK) | USBFS_UEP_R_RES_STALL; break;
                            case (UEP_IN | UEP3):
                                USBFSD->UEP3_CTRL_H = (USBFSD->UEP3_CTRL_H & ~USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_STALL; break;
                            default: errflag = 0xFF; break;
                        }
                    }
                    else errflag = 0xFF;
                }
                else errflag = 0xFF;
                break;

            case USB_GET_INTERFACE:
                ep0_[0] = 0x00;
                if (setupLen_ > 1) setupLen_ = 1;
                break;

            case USB_SET_INTERFACE:
                break;

            case USB_GET_STATUS:
                ep0_[0] = 0x00;
                ep0_[1] = 0x00;
                if ((setupReqType_ & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_DEVICE)
                {
                    if (sleepStatus_ & 0x01) ep0_[0] = 0x02;
                }
                else if ((setupReqType_ & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_ENDP)
                {
                    switch ((uint8_t)(setupIndex_ & 0xFF))
                    {
                        case (UEP_IN | UEP1):
                            if ((USBFSD->UEP1_CTRL_H & USBFS_UEP_T_RES_MASK) == USBFS_UEP_T_RES_STALL) ep0_[0] = 0x01;
                            break;
                        case (UEP_OUT | UEP2):
                            if ((USBFSD->UEP2_CTRL_H & USBFS_UEP_R_RES_MASK) == USBFS_UEP_R_RES_STALL) ep0_[0] = 0x01;
                            break;
                        case (UEP_IN | UEP3):
                            if ((USBFSD->UEP3_CTRL_H & USBFS_UEP_T_RES_MASK) == USBFS_UEP_T_RES_STALL) ep0_[0] = 0x01;
                            break;
                        default: errflag = 0xFF; break;
                    }
                }
                else errflag = 0xFF;
                if (setupLen_ > 2) setupLen_ = 2;
                break;

            default:
                errflag = 0xFF;
                break;
        }
    }

    if (errflag == 0xFF)
    {
        USBFSD->UEP0_CTRL_H = USBFS_UEP_T_TOG | USBFS_UEP_T_RES_STALL | USBFS_UEP_R_TOG | USBFS_UEP_R_RES_STALL;
    }
    else if (setupReqType_ & UEP_IN)
    {
        // device -> host data stage
        len = (setupLen_ > kEp0Size) ? kEp0Size : setupLen_;
        setupLen_ -= len;
        USBFSD->UEP0_TX_LEN = len;
        USBFSD->UEP0_CTRL_H = (USBFSD->UEP0_CTRL_H & ~USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_TOG | USBFS_UEP_T_RES_ACK;
    }
    else
    {
        // host -> device (or no data): arm status / OUT
        if (setupLen_ == 0)
        {
            USBFSD->UEP0_TX_LEN = 0;
            USBFSD->UEP0_CTRL_H = (USBFSD->UEP0_CTRL_H & ~USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_TOG | USBFS_UEP_T_RES_ACK;
        }
        else
        {
            USBFSD->UEP0_CTRL_H = (USBFSD->UEP0_CTRL_H & ~USBFS_UEP_R_RES_MASK) | USBFS_UEP_R_TOG | USBFS_UEP_R_RES_ACK;
        }
    }
} // UsbCdc::handleSetup()

// ------------------------------------------------------------------
// EP0 IN (continuation of a multi-packet control read)
// ------------------------------------------------------------------
void UsbCdc::handleEp0In()
{
    if (setupLen_ == 0)
    {
        USBFSD->UEP0_CTRL_H = (USBFSD->UEP0_CTRL_H & ~USBFS_UEP_R_RES_MASK) | USBFS_UEP_R_TOG | USBFS_UEP_R_RES_ACK;
    }

    if ((setupReqType_ & USB_REQ_TYP_MASK) == USB_REQ_TYP_STANDARD)
    {
        switch (setupReqCode_)
        {
            case USB_GET_DESCRIPTOR:
            {
                uint16_t len = (setupLen_ >= kEp0Size) ? kEp0Size : setupLen_;
                memcpy(ep0_, descr_, len);
                setupLen_ -= len;
                descr_    += len;
                USBFSD->UEP0_TX_LEN = len;
                USBFSD->UEP0_CTRL_H ^= USBFS_UEP_T_TOG;
                break;
            }
            case USB_SET_ADDRESS:
                USBFSD->DEV_ADDR = (USBFSD->DEV_ADDR & USBFS_UDA_GP_BIT) | devAddr_;
                break;
            default:
                break;
        }
    }
}

// ------------------------------------------------------------------
// EP0 OUT (data stage of a control write, e.g. SET_LINE_CODING)
// ------------------------------------------------------------------
void UsbCdc::handleEp0Out()
{
    if ((setupReqType_ & USB_REQ_TYP_MASK) != USB_REQ_TYP_STANDARD)
    {
        if (setupReqCode_ == CDC_SET_LINE_CODING)
        {
            // 7 bytes: baud(4 LE), stopBits, parity, dataBits
            for (uint8_t i = 0; i < 7; i++) lcWire_[i] = ep0_[i];
            decodeLineCoding();
            lcChanged_ = true;
        }
        setupLen_ = 0;
    }

    if (setupLen_ == 0)
    {
        USBFSD->UEP0_TX_LEN = 0;
        USBFSD->UEP0_CTRL_H = (USBFSD->UEP0_CTRL_H & ~USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_TOG | USBFS_UEP_T_RES_ACK;
    }
}

// ------------------------------------------------------------------
// Bus reset
// ------------------------------------------------------------------
void UsbCdc::busReset()
{
    devConfig_   = 0;
    devAddr_     = 0;
    sleepStatus_ = 0;
    enumerated_  = false;

    rx_.clear();
    tx_.clear();

    USBFSD->DEV_ADDR = 0;
    endpointInit();
}

// ------------------------------------------------------------------
// Interrupt dispatch
// ------------------------------------------------------------------
void UsbCdc::onIrq()
{
    uint8_t intflag = USBFSD->INT_FG;
    uint8_t intst   = USBFSD->INT_ST;

    if (intflag & USBFS_UIF_TRANSFER)
    {
        switch (intst & USBFS_UIS_TOKEN_MASK)
        {
            case USBFS_UIS_TOKEN_IN:
                switch (intst & (USBFS_UIS_TOKEN_MASK | USBFS_UIS_ENDP_MASK))
                {
                    case (USBFS_UIS_TOKEN_IN | UEP0):
                        handleEp0In();
                        break;

                    case (USBFS_UIS_TOKEN_IN | UEP1):
                        USBFSD->UEP1_CTRL_H ^= USBFS_UEP_T_TOG;
                        USBFSD->UEP1_CTRL_H = (USBFSD->UEP1_CTRL_H & ~USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_NAK;
                        break;

                    case (USBFS_UIS_TOKEN_IN | UEP3):
                        USBFSD->UEP3_CTRL_H ^= USBFS_UEP_T_TOG;
                        USBFSD->UEP3_CTRL_H = (USBFSD->UEP3_CTRL_H & ~USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_NAK;
                        ep3Busy_ = false;
                        txPkts_++;            // a bulk-IN packet was delivered to the host
                        break;

                    default:
                        break;
                }
                break;

            case USBFS_UIS_TOKEN_OUT:
                switch (intst & (USBFS_UIS_TOKEN_MASK | USBFS_UIS_ENDP_MASK))
                {
                    case (USBFS_UIS_TOKEN_OUT | UEP0):
                        if (intst & USBFS_UIS_TOG_OK) handleEp0Out();
                        break;

                    case (USBFS_UIS_TOKEN_OUT | UEP2):
                    {
                        USBFSD->UEP2_CTRL_H ^= USBFS_UEP_R_TOG;
                        uint16_t n = USBFSD->RX_LEN;
                        rx_.push(ep2_, n);
                        // Throttle the host if we can't guarantee room for the next packet.
                        if (rx_.free() < usb_desc::kPacket)
                        {
                            USBFSD->UEP2_CTRL_H = (USBFSD->UEP2_CTRL_H & ~USBFS_UEP_R_RES_MASK) | USBFS_UEP_R_RES_NAK;
                            ep2Nak_ = true;
                        }
                        break;
                    }

                    default:
                        break;
                }
                break;

            case USBFS_UIS_TOKEN_SETUP:
                handleSetup();
                break;

            case USBFS_UIS_TOKEN_SOF:
                break;

            default:
                break;
        }
        USBFSD->INT_FG = USBFS_UIF_TRANSFER;
    }
    else if (intflag & USBFS_UIF_BUS_RST)
    {
        busReset();
        USBFSD->INT_FG = USBFS_UIF_BUS_RST;
    }
    else if (intflag & USBFS_UIF_SUSPEND)
    {
        USBFSD->INT_FG = USBFS_UIF_SUSPEND;
        Delay_Us(10);
        if (USBFSD->MIS_ST & USBFS_UMS_SUSPEND) sleepStatus_ |= 0x02;
        else                                    sleepStatus_ &= ~0x02;
    }
    else
    {
        USBFSD->INT_FG = intflag;
    }
} // UsbCdc::onIrq()

// ------------------------------------------------------------------
// C-linkage ISR: forwards to the single instance.
// ------------------------------------------------------------------
extern "C" void USBFS_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
extern "C" void USBFS_IRQHandler(void)
{
    // USB ISR is DISABLED (see init()) — USB is polled from tick() instead.
    // This handler is kept for the vector table but should never fire. If it
    // does, something is wrong. 
    UsbCdc::handleIrq();
}
