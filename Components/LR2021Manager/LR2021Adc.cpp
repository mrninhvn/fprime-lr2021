// ======================================================================
// \title  LR2021Adc.cpp
// \author v.ninhdh4
// \brief  RF power monitoring of the LR2021 antenna line.
//
// The board taps the LR2021 antenna trace with a PCB coupled-line
// directional coupler; its two ports feed a pair of LT5538 logarithmic RF
// power detectors whose DC outputs go to MCU ADC inputs (devicetree
// zephyr,user channels "lr2021_fwd" and "lr2021_refl"). This file
// reads those channels through the Zephyr ADC API and converts the
// voltages to dBm.
//
// This is the only Zephyr-specific file of the component: on targets
// without the zephyr,user ADC channels readRfPower() reports failure and
// no telemetry is produced.
// ======================================================================

#include "Components/LR2021Manager/LR2021Manager.hpp"
#include <cstdlib>
#include <cmath>

#ifdef __ZEPHYR__
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>

#if DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channels)
#define LR2021_HAS_RF_ADC 1
#endif
#endif  // __ZEPHYR__

namespace LR2021 {

namespace {

// ---------------------------------------------------------------------------
// LT5538 log-detector transfer function: Vout = (P_in - INTERCEPT) * SLOPE.
// Datasheet 2140 MHz
// ---------------------------------------------------------------------------
#ifndef LT5538_SLOPE_MV_PER_DB
#define LT5538_SLOPE_MV_PER_DB 17.7f
#endif

#ifndef LT5538_INTERCEPT_DBM
#define LT5538_INTERCEPT_DBM (-89.0f)
#endif

// Coupling factor of the PCB directional coupler, in dB: add this to the
// detector reading to get the power on the antenna line itself. 0 reports
// raw power at the detector input; set after characterising the coupler.
#ifndef LR2021_COUPLER_LOSS_DB
#define LR2021_COUPLER_LOSS_DB 0.0f
#endif

// Delay between set_tx and the detector read, in ms: covers the PA ramp
// and the LT5538 output filter (100 nF) settling. Must stay well below the
// shortest TX airtime (FLRC 6-byte packet at 0.65 Mbps is ~2.5 ms with
// preamble/syncword, a TM frame at 9600 bps is ~200 ms).
#ifndef LR2021_RF_POWER_SETTLE_MS
#define LR2021_RF_POWER_SETTLE_MS 1
#endif

#ifdef LR2021_HAS_RF_ADC

// Detector channels (see the zephyr,user node in the board devicetree).
// Both are on the same ADC instance (adc1), which is what lets a single
// sequence sample them together.
const struct adc_dt_spec kFwdAdc = ADC_DT_SPEC_GET_BY_NAME(DT_PATH(zephyr_user), lr2021_fwd);
const struct adc_dt_spec kReflAdc = ADC_DT_SPEC_GET_BY_NAME(DT_PATH(zephyr_user), lr2021_refl);

//! Convert an LT5538 detector output voltage (mV) to power in dBm.
F32 dbmFromMv(I32 mv) {
    return static_cast<F32>(mv) / LT5538_SLOPE_MV_PER_DB + LT5538_INTERCEPT_DBM +
           LR2021_COUPLER_LOSS_DB;
}

#endif  // LR2021_HAS_RF_ADC

}  // namespace

// ----------------------------------------------------------------------
// RF power detectors
// ----------------------------------------------------------------------

bool LR2021Manager ::readRfPower(F32& fwd_dbm, F32& refl_dbm) {
#ifdef LR2021_HAS_RF_ADC
    if (!this->m_adcReady) {
        // A single multi-channel sequence only works if both detectors are
        // on the same ADC instance; the board wires them both to adc1.
        if (kFwdAdc.dev != kReflAdc.dev) {
            return false;
        }
        if (!adc_is_ready_dt(&kFwdAdc) || !adc_is_ready_dt(&kReflAdc)) {
            return false;
        }
        if ((adc_channel_setup_dt(&kFwdAdc) != 0) || (adc_channel_setup_dt(&kReflAdc) != 0)) {
            return false;
        }
        this->m_adcReady = true;
    }

    // Read both detectors in one sequence: the forward and reflected
    // channels are converted back-to-back in a single triggered scan, so
    // the two samples are captured within a few microseconds of each other
    // (versus two separate adc_read() calls straddling the whole TX burst).
    U16 raw[2] = {0, 0};
    struct adc_sequence sequence = {};
    (void)adc_sequence_init_dt(&kFwdAdc, &sequence);
    sequence.channels |= BIT(kReflAdc.channel_id);
    sequence.buffer = raw;
    sequence.buffer_size = sizeof raw;
    if (adc_read_dt(&kFwdAdc, &sequence) != 0) {
        return false;
    }

    // Samples land in the buffer in ascending channel-id order, so map each
    // one back to its detector rather than assuming a fixed position.
    const bool fwdFirst = kFwdAdc.channel_id < kReflAdc.channel_id;
    I32 fwd_mv = fwdFirst ? raw[0] : raw[1];
    I32 refl_mv = fwdFirst ? raw[1] : raw[0];
    if ((adc_raw_to_millivolts_dt(&kFwdAdc, &fwd_mv) != 0) ||
        (adc_raw_to_millivolts_dt(&kReflAdc, &refl_mv) != 0)) {
        return false;
    }
    DEBUG("ADC fwd=%d mV refl=%d mV", fwd_mv, refl_mv);

    fwd_dbm = dbmFromMv(fwd_mv);
    refl_dbm = dbmFromMv(refl_mv);
    return true;
#else
    (void)fwd_dbm;
    (void)refl_dbm;
    return false;
#endif
}

void LR2021Manager ::rfPowerMeasureTx() {
    // Let the PA ramp up and the detector output filters settle before
    // sampling: the TX preamble is already on the air at this point.
    this->delayMs(LR2021_RF_POWER_SETTLE_MS);

    F32 fwd_dbm = 0.0f;
    F32 refl_dbm = 0.0f;
    if (this->readRfPower(fwd_dbm, refl_dbm)) {
        LR2021::RfPower power;
        power.set_FwdDbmX10(static_cast<I16>(std::round(fwd_dbm * 10.0f)));
        power.set_ReflDbmX10(static_cast<I16>(std::round(refl_dbm * 10.0f)));
        const I16 fwdX10 = power.get_FwdDbmX10();
        const I16 reflX10 = power.get_ReflDbmX10();
        DEBUG("fwd=%d.%d refl=%d.%d dBm", fwdX10 / 10, abs(fwdX10 % 10), reflX10 / 10, abs(reflX10 % 10));
        this->tlmWrite_RfPower(power);
    }
}

}  // namespace LR2021
