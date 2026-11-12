// ======================================================================
// \title  LR2021TxCycle.cpp
// \author ninhdh4
// \brief  TX cycle of the LR2021Manager component (Sat3/Sat4): once RF is
//         allowed (RadioTxEnable TRUE, sent by the MissionController LEOP),
//         the downlink goes round the radios:
//
//   NICERF       radio 0 on its link, every downlink to it, radio 1 off   LR2021_TX_CYCLE_NICERF_S
//   REYAX_UHF    radio 1 on the link of radio 0, downlink to it, 0 off    LR2021_TX_CYCLE_REYAX_UHF_S
//   REYAX_SBAND  radio 1 on its own link                                  LR2021_TX_CYCLE_SBAND_S
//   then NICERF again.
//
// The links and routes are the PRM_COM_* parameters, read at every phase and
// never written; one not stored keeps what the radio had when the cycle
// started (the boot setup of the topology). PRM_COM_TX_CYCLE carries the
// cycle on after a reset. The first ground command (PRM_MODE_LAST_TIME_RX_COM,
// stamped by UplinkMonitor, changes) or RadioTxEnable FALSE ends it: both
// radios back on their links, the routes back. Without the parameter
// database (prmGetOut / prmSetOut unconnected) there is no cycle.
// ======================================================================

#include "Components/LR2021Manager/LR2021Manager.hpp"
#include "Components/LR2021Manager/LR2021Cfg.hpp"

// The PrmId enum this project autocodes (PrmIds.fpp); without it, no TX cycle
#if __has_include("FprimeZephyrReference/project/config/PrmIdEnumAc.hpp")
#include "FprimeZephyrReference/project/config/PrmIdEnumAc.hpp"
#define LR2021_TX_CYCLE_PRM_IDS 1
#else
#define LR2021_TX_CYCLE_PRM_IDS 0
#endif

namespace LR2021 {

#if LR2021_TX_CYCLE_PRM_IDS
namespace {
// PRM_COM_RADIOn_* of each radio: mode, TX frequency, RX frequency, power
const FwPrmIdType LINK_PRM[2][4] = {
    {Vspc::PrmId::PRM_COM_RADIO0_MODE, Vspc::PrmId::PRM_COM_RADIO0_TX_FREQ, Vspc::PrmId::PRM_COM_RADIO0_RX_FREQ,
     Vspc::PrmId::PRM_COM_RADIO0_POWER},
    {Vspc::PrmId::PRM_COM_RADIO1_MODE, Vspc::PrmId::PRM_COM_RADIO1_TX_FREQ, Vspc::PrmId::PRM_COM_RADIO1_RX_FREQ,
     Vspc::PrmId::PRM_COM_RADIO1_POWER}};
// PRM_COM_ROUTE_* of EVT, TLM, FILE
const FwPrmIdType ROUTE_PRM[3] = {Vspc::PrmId::PRM_COM_ROUTE_EVT, Vspc::PrmId::PRM_COM_ROUTE_TLM,
                                  Vspc::PrmId::PRM_COM_ROUTE_FILE};
}  // namespace
#endif

void LR2021Manager ::RadioTxEnable_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, bool enable) {
    if (this->txCycleAvailable()) {
        if (enable) {
            this->txCycleSaveRunning(true);
            this->txCycleStart();
        } else if (this->m_txPhase != LR2021Manager_TxCyclePhase::OFF) {
            this->txCycleStop();
        }
    }
    // Without the parameter database the radios already transmit: nothing to do
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

bool LR2021Manager ::txCycleAvailable() {
    return (LR2021_TX_CYCLE_PRM_IDS != 0) && this->isConnected_prmGetOut_OutputPort(0) &&
           this->isConnected_prmSetOut_OutputPort(0);
}

template <typename T>
bool LR2021Manager ::txCyclePrm(FwPrmIdType id, T& value) {
    Fw::ParamBuffer buffer;
    buffer.resetSer();
    if (this->prmGetOut_out(0, id, buffer) != Fw::ParamValid::VALID) {
        return false;
    }
    T read;
    buffer.resetDeser();
    if (buffer.deserializeTo(read) != Fw::FW_SERIALIZE_OK) {
        return false;
    }
    value = read;
    return true;
}

void LR2021Manager ::txCycleSaveRunning(bool running) {
#if LR2021_TX_CYCLE_PRM_IDS
    Fw::ParamBuffer buffer;
    buffer.resetSer();
    const U8 value = running ? 1U : 0U;
    if (buffer.serializeFrom(value) == Fw::FW_SERIALIZE_OK) {
        this->prmSetOut_out(0, Vspc::PrmId::PRM_COM_TX_CYCLE, buffer);
    }
#else
    static_cast<void>(running);
#endif
}

void LR2021Manager ::txCycleBoot() {
#if LR2021_TX_CYCLE_PRM_IDS
    U8 running = 0U;
    if (this->txCycleAvailable() && this->txCyclePrm(Vspc::PrmId::PRM_COM_TX_CYCLE, running) && (running != 0U)) {
        this->txCycleStart();
    }
#endif
}

void LR2021Manager ::txCycleStart() {
#if LR2021_TX_CYCLE_PRM_IDS
    // A restart keeps the fallbacks of the cycle running: the radios are not on their links now
    if (this->m_txPhase == LR2021Manager_TxCyclePhase::OFF) {
        for (FwIndexType i = 0; i < NUM_RADIOS; i++) {
            const RadioSlot& r = this->m_radio[i];
            TxLink& link = this->m_txFallback[i];
            // A timed TX test parks r.mode: take the mode it restores
            link.mode = r.txTesting ? r.txTestMode : r.mode;
            link.powerDbm = r.txTesting ? r.txTestPowerDbm : r.powerDbm;
            // The frequencies the radio is on: its own TX / RX ones, else the base one of the mode
            const U32 base = r.txTesting ? r.txTestFreqHz : r.freqHz;
            link.txFreqHz = (r.txFreqHz != 0U) ? r.txFreqHz : base;
            link.rxFreqHz = (r.rxFreqHz != 0U) ? r.rxFreqHz : base;
        }
        for (FwIndexType q = 0; q < TX_CYCLE_QUEUES; q++) {
            this->m_txFallbackRoute[q] = this->m_txRoute[q];
        }
    }

    // A ground command newer than the last one stored now ends the cycle
    this->m_txCycleTimeRxValid = this->txCyclePrm(Vspc::PrmId::PRM_MODE_LAST_TIME_RX_COM, this->m_txCycleTimeRx);

    this->txCycleEnter(LR2021Manager_TxCyclePhase::NICERF);
#endif
}

void LR2021Manager ::txCycleStop() {
    this->txCycleEnter(LR2021Manager_TxCyclePhase::OFF);
    this->txCycleSaveRunning(false);
}

void LR2021Manager ::txCycleService() {
#if LR2021_TX_CYCLE_PRM_IDS
    if (this->m_txPhase == LR2021Manager_TxCyclePhase::OFF) {
        return;
    }
    if (++this->m_txServiceTicks < LR2021_TX_CYCLE_SERVICE_TICKS) {
        return;
    }
    this->m_txServiceTicks = 0;

    // First ground command since the start
    U32 lastRx = 0U;
    if (this->txCyclePrm(Vspc::PrmId::PRM_MODE_LAST_TIME_RX_COM, lastRx) &&
        (!this->m_txCycleTimeRxValid || (lastRx != this->m_txCycleTimeRx))) {
        this->txCycleStop();
        return;
    }

    // Time of the phase
    Os::RawTime now;
    Fw::TimeInterval elapsed;
    if ((now.now() != Os::RawTime::OP_OK) ||
        (now.getTimeInterval(this->m_txPhaseStart, elapsed) != Os::RawTime::OP_OK)) {
        return;
    }
    const U32 seconds = elapsed.getSeconds();
    switch (this->m_txPhase.e) {
        case LR2021Manager_TxCyclePhase::NICERF:
            if (seconds >= LR2021_TX_CYCLE_NICERF_S) {
                this->txCycleEnter(LR2021Manager_TxCyclePhase::REYAX_UHF);
            }
            break;
        case LR2021Manager_TxCyclePhase::REYAX_UHF:
            if (seconds >= LR2021_TX_CYCLE_REYAX_UHF_S) {
                this->txCycleEnter(LR2021Manager_TxCyclePhase::REYAX_SBAND);
            }
            break;
        case LR2021Manager_TxCyclePhase::REYAX_SBAND:
            if (seconds >= LR2021_TX_CYCLE_SBAND_S) {
                this->txCycleEnter(LR2021Manager_TxCyclePhase::NICERF);
            }
            break;
        default:
            break;
    }
#endif
}

LR2021Manager::TxLink LR2021Manager ::txCycleLink(FwIndexType idx) {
    TxLink link = this->m_txFallback[idx];
#if LR2021_TX_CYCLE_PRM_IDS
    U8 mode = 0U;
    if (this->txCyclePrm(LINK_PRM[idx][0], mode)) {
        // LR2021Manager.Mode values; any other keeps the fallback mode
        if (mode == static_cast<U8>(LR2021Manager_Mode::FLRC)) {
            link.mode = RadioMode::FLRC;
        } else if (mode == static_cast<U8>(LR2021Manager_Mode::FSK)) {
            link.mode = RadioMode::FSK;
        }
    }
    // A frequency stored as 0 is none: the fallback stays
    U32 freq = 0U;
    if (this->txCyclePrm(LINK_PRM[idx][1], freq) && (freq != 0U)) {
        link.txFreqHz = freq;
    }
    if (this->txCyclePrm(LINK_PRM[idx][2], freq) && (freq != 0U)) {
        link.rxFreqHz = freq;
    }
    static_cast<void>(this->txCyclePrm(LINK_PRM[idx][3], link.powerDbm));
#endif
    return link;
}

bool LR2021Manager ::txCycleApply(FwIndexType idx, const TxLink& link) {
    RadioSlot& r = this->m_radio[idx];
    // A running BER test keeps driving its radios until it finishes
    if (this->m_ber.active && ((idx == this->m_ber.txRadio) || (idx == this->m_ber.rxRadio))) {
        return false;
    }
    // One RF path per mode: a link whose frequencies are in two bands cannot be set
    if ((link.mode == RadioMode::NONE) || (link.txFreqHz == 0U) || (link.rxFreqHz == 0U) ||
        (isHfBand(link.txFreqHz) != isHfBand(link.rxFreqHz))) {
        return false;
    }
    if (r.poweredOff && !this->isConnected_powerGpioWrite_OutputPort(idx)) {
        return false;
    }
    // The TX and RX frequencies first: the mode set below tunes with them. Its base
    // frequency (band, PA / RX path) is the TX one, as RadioSetMode
    r.txFreqHz = link.txFreqHz;
    r.rxFreqHz = link.rxFreqHz;
    if (r.poweredOff) {
        // radioPowerOn() restores the slot's mode: give it the link's
        r.mode = link.mode;
        r.freqHz = link.txFreqHz;
        r.powerDbm = link.powerDbm;
        return this->radioPowerOn(r);
    }
    return this->setMode(idx, link.mode, link.txFreqHz, link.powerDbm);
}

void LR2021Manager ::txCycleEnter(LR2021Manager_TxCyclePhase phase) {
    const FwIndexType NICERF = 0;
    const FwIndexType REYAX = 1;
    const TxLink link0 = this->txCycleLink(NICERF);
    const TxLink link1 = this->txCycleLink(REYAX);

    // The radio of the phase on, every queue to it, then the other one off
    FwIndexType on = NICERF;
    FwIndexType off = REYAX;
    bool ok = true;
    switch (phase.e) {
        case LR2021Manager_TxCyclePhase::NICERF:
            if (!this->txCycleApply(NICERF, link0)) {
                this->log_WARNING_HI_TxCycleRadioError(static_cast<U8>(NICERF), phase);
                ok = false;
            }
            break;
        case LR2021Manager_TxCyclePhase::REYAX_UHF:
            on = REYAX;
            off = NICERF;
            if (!this->txCycleApply(REYAX, link0)) {
                this->log_WARNING_HI_TxCycleRadioError(static_cast<U8>(REYAX), phase);
                ok = false;
            }
            break;
        case LR2021Manager_TxCyclePhase::REYAX_SBAND:
            on = REYAX;
            off = NICERF;
            if (!this->txCycleApply(REYAX, link1)) {
                this->log_WARNING_HI_TxCycleRadioError(static_cast<U8>(REYAX), phase);
                ok = false;
            }
            break;
        case LR2021Manager_TxCyclePhase::OFF:
        default: {
            // Both radios on their links, the default routes
            if (!this->txCycleApply(NICERF, link0)) {
                this->log_WARNING_HI_TxCycleRadioError(static_cast<U8>(NICERF), phase);
            }
            if (!this->txCycleApply(REYAX, link1)) {
                this->log_WARNING_HI_TxCycleRadioError(static_cast<U8>(REYAX), phase);
            }
            for (FwIndexType q = 0; q < TX_CYCLE_QUEUES; q++) {
                FwIndexType target = this->m_txFallbackRoute[q];
#if LR2021_TX_CYCLE_PRM_IDS
                U8 stored = 0U;
                if (this->txCyclePrm(ROUTE_PRM[q], stored) && (stored <= DISABLED_RADIO)) {
                    target = static_cast<FwIndexType>(stored);
                }
#endif
                this->setTxRoute(q, target);
            }
            this->m_txPhase = LR2021Manager_TxCyclePhase::OFF;
            this->log_ACTIVITY_HI_TxCycleSet(this->m_txPhase);
            return;
        }
    }

    // Downlink to the radio of the phase only once it is up; the other one is cut either way
    if (ok) {
        for (FwIndexType q = 0; q < TX_CYCLE_QUEUES; q++) {
            this->setTxRoute(q, on);
        }
    }
    RadioSlot& other = this->m_radio[off];
    const bool otherBusy = this->m_ber.active && ((off == this->m_ber.txRadio) || (off == this->m_ber.rxRadio));
    if (ok && !other.poweredOff && !otherBusy && this->isConnected_powerGpioWrite_OutputPort(off)) {
        this->radioPowerOff(other);
    }

    this->m_txPhase = phase;
    static_cast<void>(this->m_txPhaseStart.now());
    this->m_txServiceTicks = 0;
    this->log_ACTIVITY_HI_TxCycleSet(this->m_txPhase);
}

}  // namespace LR2021
