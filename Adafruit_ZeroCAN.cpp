/*!
 * @file Adafruit_ZeroCAN.cpp
 *
 * @mainpage Adafruit CAN peripheral driver for SAME51 chips
 *
 * @section intro_sec Introduction
 *
 *  CAN peripheral driver for SAME51 chips
 *
 *
 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing
 *  products from Adafruit!
 *
 * @section author Author
 *
 * Written by Jeff Epler for Adafruit Industries.
 *
 * @section license License
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

#include <algorithm>

#include <stdint.h>
#include <stdlib.h>

#include "Adafruit_ZeroCAN.h"
#include "wiring_private.h"
 
#include "same51.h"

#define hw (reinterpret_cast<Can*>(this->_hw))

#define DIV_ROUND(a, b) (((a) + (b)/2) / (b))
#define DIV_ROUND_UP(a, b) (((a) + (b) - 1) / (b))

#define GCLK_CAN1 GCLK_PCHCTRL_GEN_GCLK1_Val
#define ADAFRUIT_ZEROCAN_TX_BUFFER_SIZE (1)
#define ADAFRUIT_ZEROCAN_RX_FILTER_SIZE (14)
#define ADAFRUIT_ZEROCAN_RX_FIFO_SIZE (8)
#define ADAFRUIT_ZEROCAN_MAX_MESSAGE_LENGTH (8)

#define CAN1_FUNCTION (EPioType(7))

// This appears to be a typo (transposition error) in the ASF4 headers
// It's called the "Extended ID Filter Entry"
typedef CanMramXifde CanMramXidfe;

typedef uint32_t can_filter_t;

struct _adafruit_ZeroCAN_tx_buf {
    CAN_TXBE_0_Type txb0;
    CAN_TXBE_1_Type txb1;
    __attribute__((aligned(4)))
    uint8_t data[8];
};

struct _adafruit_ZeroCAN_rx_fifo {
    CAN_RXF0E_0_Type rxf0;
    CAN_RXF0E_1_Type rxf1;
    __attribute((aligned(4)))
    uint8_t data[ADAFRUIT_ZEROCAN_MAX_MESSAGE_LENGTH];
} can_rx_fifo_t;

struct _adafruit_ZeroCAN_state {
    _adafruit_ZeroCAN_tx_buf tx_buffer[ADAFRUIT_ZEROCAN_TX_BUFFER_SIZE];
    _adafruit_ZeroCAN_rx_fifo rx0_fifo[ADAFRUIT_ZEROCAN_RX_FIFO_SIZE];
    _adafruit_ZeroCAN_rx_fifo rx1_fifo[ADAFRUIT_ZEROCAN_RX_FIFO_SIZE];
    CanMramSidfe standard_rx_filter[ADAFRUIT_ZEROCAN_RX_FILTER_SIZE];
    CanMramXifde extended_rx_filter[ADAFRUIT_ZEROCAN_RX_FILTER_SIZE];
};

namespace {
    // This data must be in the first 64kB of RAM.  The "canram" section
    // receives special support from the linker file in the Feather M4 CAN's
    // board support package.
    // TODO support CAN0 and CAN1 simultaneously (state would be an array of 2)
    __attribute__((section("canram")))
    _adafruit_ZeroCAN_state can_state;

    constexpr uint32_t can_frequency = VARIANT_GCLK1_FREQ;
    bool compute_nbtp(uint32_t baudrate, CAN_NBTP_Type &result) {
        uint32_t clocks_per_bit = DIV_ROUND(can_frequency, baudrate);
        uint32_t clocks_to_sample = DIV_ROUND(clocks_per_bit * 7, 8);
        uint32_t clocks_after_sample = clocks_per_bit - clocks_to_sample;
        uint32_t divisor = std::max(DIV_ROUND_UP(clocks_to_sample, 256), DIV_ROUND_UP(clocks_after_sample, 128));
        if (divisor > 32) {
            return false;
        }
        result.bit.NTSEG1 = DIV_ROUND(clocks_to_sample, divisor) - 2;
        result.bit.NTSEG2 = DIV_ROUND(clocks_after_sample, divisor) - 2;
        result.bit.NBRP = divisor - 1;
        result.bit.NSJW = DIV_ROUND(clocks_after_sample, divisor * 4);
        return true;
    }
}

Adafruit_ZeroCAN::Adafruit_ZeroCAN(uint8_t TX_PIN, uint8_t RX_PIN) : _tx(TX_PIN), _rx(RX_PIN) {}
#ifdef PIN_CAN_TX
Adafruit_ZeroCAN::Adafruit_ZeroCAN() : _tx(PIN_CAN_TX), _rx(PIN_CAN_RX) {}
#else
Adafruit_ZeroCAN::Adafruit_ZeroCAN() : _tx(-1) {}
#endif

bool Adafruit_ZeroCAN::begin(int baudrate, bool loopback, bool silent) {
    if (_tx == -1) {
        return false;
    }
    
    CAN_NBTP_Type nbtp;
    if (!compute_nbtp(baudrate, nbtp)) {
        return false;
    }

    // TODO: Support the CAN0 peripheral, which uses pinmux 8
    _hw = reinterpret_cast<void*>(CAN1);
    state = &can_state;
    memset(state, 0, sizeof(*state));

    pinPeripheral(_tx, CAN1_FUNCTION);
    pinPeripheral(_rx, CAN1_FUNCTION);

    GCLK->PCHCTRL[CAN1_GCLK_ID].reg = GCLK_CAN1 | (1 << GCLK_PCHCTRL_CHEN_Pos);

    // reset and allow configuration change
    hw->CCCR.bit.INIT = 1;
    while (!hw->CCCR.bit.INIT) {
    }
    hw->CCCR.bit.CCE = 1;
    
    // set loopback and silent modes
    hw->CCCR.bit.MON = silent;
    hw->CCCR.bit.TEST = loopback;
    hw->TEST.bit.LBCK = loopback;

    // All TX data has an 8 byte payload (max)
    {
        CAN_TXESC_Type esc = {};
        esc.bit.TBDS = CAN_TXESC_TBDS_DATA8_Val;
        CAN1->TXESC.reg = esc.reg;
    }

    // Set up TX buffer
    {
        CAN_TXBC_Type bc = {};
        bc.bit.TBSA = (uint32_t)state->tx_buffer;
        bc.bit.NDTB = ADAFRUIT_ZEROCAN_TX_BUFFER_SIZE;
        bc.bit.TFQM = 0; // Messages are transmitted in the order submitted
        CAN1->TXBC.reg = bc.reg;
    }

    // All RX data has an 8 byte payload (max)
    {
        CAN_RXESC_Type esc = {};
        esc.bit.F0DS = CAN_RXESC_F0DS_DATA8_Val;
        esc.bit.F1DS = CAN_RXESC_F1DS_DATA8_Val;
        esc.bit.RBDS = CAN_RXESC_RBDS_DATA8_Val;
        hw->RXESC.reg = esc.reg;
    }

    // Set up RX fifo 0
    {
        CAN_RXF0C_Type rxf = {};
        rxf.bit.F0SA = (uint32_t)state->rx0_fifo;
        rxf.bit.F0S = ADAFRUIT_ZEROCAN_RX_FIFO_SIZE;
        hw->RXF0C.reg = rxf.reg;
    }

    // Set up RX fifo 1
    {
        CAN_RXF1C_Type rxf = {};
        rxf.bit.F1SA = (uint32_t)state->rx1_fifo;
        rxf.bit.F1S = ADAFRUIT_ZEROCAN_RX_FIFO_SIZE;
        hw->RXF1C.reg = rxf.reg;
    }

    // Reject all packets not explicitly requested
    {
        CAN_GFC_Type gfc = {};
        gfc.bit.RRFE = 0;
        gfc.bit.ANFS = CAN_GFC_ANFS_REJECT_Val;
        gfc.bit.ANFE = CAN_GFC_ANFE_REJECT_Val;
        hw->GFC.reg = gfc.reg;
    }

    // Set up standard RX filters
    {
        CAN_SIDFC_Type dfc = {};
        dfc.bit.LSS = ADAFRUIT_ZEROCAN_RX_FILTER_SIZE;
        dfc.bit.FLSSA = (uint32_t)state->standard_rx_filter;
        hw->SIDFC.reg = dfc.reg;
    }

    // Set up extended RX filters
    {
        CAN_XIDFC_Type dfc = {};
        dfc.bit.LSE = ADAFRUIT_ZEROCAN_RX_FILTER_SIZE;
        dfc.bit.FLESA = (uint32_t)state->extended_rx_filter;
        hw->XIDFC.reg = dfc.reg;
    }

    // Set nominal baud rate
    hw->NBTP.reg = nbtp.reg;

    // hardware is ready for use
    CAN1->CCCR.bit.CCE = 0;
    CAN1->CCCR.bit.INIT = 0;
    while(CAN1->CCCR.bit.INIT) {
    }

    return true;
}

int Adafruit_ZeroCAN::transmitErrorCount() const {
    return hw->ECR.bit.TEC;
}

int Adafruit_ZeroCAN::receiveErrorCount() const {
    return hw->ECR.bit.REC;
}

Adafruit_ZeroCAN::BusState Adafruit_ZeroCAN::busState() const {
    CAN_PSR_Type psr;
    psr.reg = hw->PSR.reg;

    if (psr.bit.BO) { return BUS_OFF; }
    if (psr.bit.EP) { return ERROR_PASSIVE; }
    if (psr.bit.EW) { return ERROR_WARNING; }
    return ERROR_ACTIVE;
}

bool Adafruit_ZeroCAN::send(const Message &m) {
    _adafruit_ZeroCAN_tx_buf &buf = state->tx_buffer[0];
    buf.txb0.bit.ESI = false;
    buf.txb0.bit.XTD = m.extended;
    buf.txb0.bit.RTR = m.rtr;
    if(m.extended) {
        buf.txb0.bit.ID = m.id;
    } else {
        buf.txb0.bit.ID = m.id << 18;
    }
    buf.txb1.bit.MM = 0;
    buf.txb1.bit.EFC = 0;
    buf.txb1.bit.FDF = 0;
    buf.txb1.bit.BRS = 0;
    buf.txb1.bit.DLC = m.size;

    if (!m.rtr) {
        memcpy(buf.data, m.data, sizeof(m.data));
    }

    // TX buffer add request
    hw->TXBAR.reg = 1;

    // wait 8ms (hard coded for now) for TX to occur
    for (int i=0; i<8000; i++) {
        if(hw->TXBTO.reg & 1) {
            return true;
        }
        delayMicroseconds(1);
    }

    return false;
}

void Adafruit_ZeroCAN::restart() {
    hw->CCCR.bit.INIT = 0;
    while(hw->CCCR.bit.INIT) {
    }
}
