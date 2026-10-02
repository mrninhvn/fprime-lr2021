/*
 * LR2021Cfg.hpp
 *
 *  Created on: July 7, 2026
 *      Author: ninhdh4
 */

#ifndef _LR2021_LR2021CFG_HPP_
#define _LR2021_LR2021CFG_HPP_

#include "Components/LR2021Manager/LR2021Manager.hpp"

extern "C" {
#include "lr20xx_radio_flrc.h"
#include "lr20xx_radio_fsk.h"
}

#ifndef FLRC_RAW_BIT_RATE
#define FLRC_RAW_BIT_RATE LR20XX_RADIO_FLRC_BR_0_650_BW_0_740
// #define FLRC_RAW_BIT_RATE LR20XX_RADIO_FLRC_BR_2_600_BW_2_666
#endif

#ifndef FLRC_CR
#define FLRC_CR LR20XX_RADIO_FLRC_CR_3_4
#endif

#ifndef FLRC_PULSE_SHAPE
#define FLRC_PULSE_SHAPE LR20XX_RADIO_FLRC_PULSE_SHAPE_BT_05
#endif

#ifndef FLRC_PREAMBLE_BITS
#define FLRC_PREAMBLE_BITS LR20XX_RADIO_FLRC_PREAMBLE_LEN_32_BITS
#endif

#ifndef FLRC_SYNCWORD_LEN
#define FLRC_SYNCWORD_LEN LR20XX_RADIO_FLRC_SYNCWORD_LENGTH_4_BYTES
#endif

#ifndef FLRC_TX_SYNCWORD
#define FLRC_TX_SYNCWORD LR20XX_RADIO_FLRC_TX_SYNCWORD_1
#endif

#ifndef FLRC_MATCH_SYNCWORD
#define FLRC_MATCH_SYNCWORD LR20XX_RADIO_FLRC_RX_MATCH_SYNCWORD_1
#endif

#ifndef FLRC_PKT_LEN_TYPE
#define FLRC_PKT_LEN_TYPE LR20XX_RADIO_FLRC_PKT_VAR_LEN
#endif

#ifndef FLRC_CRC
// #define FLRC_CRC LR20XX_RADIO_FLRC_CRC_2_BYTES
#define FLRC_CRC LR20XX_RADIO_FLRC_CRC_OFF
#endif

// Raw over-the-air bit rate (bps) for the selected FLRC_RAW_BIT_RATE, used to
// report the link speed (e.g. in BER test results). Kept in sync with the
// br_bw enum here so it tracks any change to FLRC_RAW_BIT_RATE at compile time.
// Single return statement (C++11 constexpr): a ternary chain over the enum.
constexpr uint32_t flrcRawBitrateBps(lr20xx_radio_flrc_br_bw_t br_bw) {
    return (br_bw == LR20XX_RADIO_FLRC_BR_2_600_BW_2_666)   ? 2600000u
           : (br_bw == LR20XX_RADIO_FLRC_BR_2_080_BW_2_222) ? 2080000u
           : (br_bw == LR20XX_RADIO_FLRC_BR_1_300_BW_1_333) ? 1300000u
           : (br_bw == LR20XX_RADIO_FLRC_BR_1_040_BW_1_333) ? 1040000u
           : (br_bw == LR20XX_RADIO_FLRC_BR_0_650_BW_0_740) ? 650000u
           : (br_bw == LR20XX_RADIO_FLRC_BR_0_520_BW_0_571) ? 520000u
           : (br_bw == LR20XX_RADIO_FLRC_BR_0_325_BW_0_357) ? 325000u
           : (br_bw == LR20XX_RADIO_FLRC_BR_0_260_BW_0_307) ? 260000u
                                                            : 0u;
}
constexpr uint32_t FLRC_RAW_BITRATE_BPS = flrcRawBitrateBps(FLRC_RAW_BIT_RATE);

// Syncword #1, used for both TX and RX matching.
constexpr uint8_t FLRC_SYNCWORD[LR20XX_RADIO_FLRC_SYNCWORD_LENGTH] = { 0x90, 0x56, 0x34, 0x12 };

// TX/RX operation timeout used when starting TX, in ms.
constexpr uint32_t FLRC_TX_TIMEOUT_MS = 5000;

// RTC-step value selecting RX continuous mode (see lr20xx_radio_common.h).
constexpr uint32_t FLRC_RX_CONTINUOUS = 0xFFFFFF;

// ---------------------------------------------------------------------------
// FSK mode defaults: GMSK per CCSDS
//
// GMSK is Gaussian-filtered continuous-phase FSK with modulation index
// h = 2 * fdev / bitrate = 0.5, i.e. fdev = bitrate / 4. CCSDS 211.1
// (Proximity-1 physical layer) specifies GMSK with BT = 0.5; CCSDS 401.0-B
// recommends BTs = 0.25 for Earth-space links (not available on the LR2021;
// GAUSSIAN_BT_0_3 is the closest).
// ---------------------------------------------------------------------------

#ifndef FSK_BITRATE_BPS
// Matched to the UHF ground segment: 2-FSK / GMSK with CCSDS up to 10 kbps.
#define FSK_BITRATE_BPS 9600
#endif

#ifndef FSK_FDEV_HZ
// GMSK condition: h = 0.5 -> fdev = bitrate / 4. Changing this breaks GMSK.
#define FSK_FDEV_HZ (FSK_BITRATE_BPS / 4)
#endif

#ifndef FSK_RX_BW
// Signal bandwidth (Carson): 2 * (FSK_FDEV_HZ + FSK_BITRATE_BPS / 2)
// = 1.5 * bitrate = 14.4 kHz. The filter margin beyond that must absorb
// the residual carrier offset; the LR2021 driver exposes no FEI/AFC, so
// the filter is the only tolerance. At 437 MHz / 550 km LEO:
//   - Doppler is +/-10.2 kHz uncompensated, ~150 Hz/s max rate;
//   - with TLE-based compensation at the ground station, the residual is
//     the TCXO error, ~+/-1 kHz per side.
// 20 kHz gives +/-2.8 kHz margin: right for a Doppler-compensated ground
// station. For an uncompensated link use RX_BW_41_000_HZ (+/-13.3 kHz
// margin, ~4 dB sensitivity cost).
#define FSK_RX_BW LR20XX_RADIO_FSK_COMMON_RX_BW_20_000_HZ
#endif

#ifndef FSK_PULSE_SHAPE
// BT = 0.5 per CCSDS 211.1 Proximity-1 GMSK
#define FSK_PULSE_SHAPE LR20XX_RADIO_FSK_PULSE_SHAPE_GAUSSIAN_BT_0_5
#endif

#ifndef FSK_PREAMBLE_BITS
#define FSK_PREAMBLE_BITS 32
#endif

#ifndef FSK_PREAMBLE_DETECTOR
// Preamble-gated detection: reduces false syncs on noise (critical for the
// short 16-bit TC syncword: syncword-only detection false-triggers about
// every 7 s at 9600 bps) and gives the AGC/bit-sync time to settle at low
// SNR. Note: the first-byte corruption once blamed on this detector was in
// fact the TX modem with a 16-bit syncword (see FSK_TC_TX_SYNCWORD).
#define FSK_PREAMBLE_DETECTOR LR20XX_RADIO_FSK_PREAMBLE_DETECTOR_16_BITS
#endif

#ifndef FSK_HEADER_MODE
// Implicit (no length header transmitted): every packet is exactly
// FSK_FIXED_PAYLOAD_LEN bytes, so the channel is pure CCSDS:
// preamble | ASM | frame. Set to LR20XX_RADIO_FSK_HEADER_8BITS for
// variable-length packets instead (SX126X / SX127X compatible).
#define FSK_HEADER_MODE LR20XX_RADIO_FSK_HEADER_IMPLICIT
#endif

// Fixed length of a TM (downlink) frame. Must equal ComCfg::TmFrameFixedSize.
// With RS coding enabled this is the RS(255,223) data field, so it must be
// 223; the packet on the air is frame + 32 RS parity bytes after the ASM.
#ifndef FSK_FIXED_PAYLOAD_LEN
#define FSK_FIXED_PAYLOAD_LEN 223
#endif

// Reed-Solomon (255,223) coding on the TM downlink (CCSDS 131.0-B). The
// ground segment decodes RS and convolutional only; under the 10 kbps
// channel cap RS is the only option that keeps full throughput (CC halves
// it, concatenated RS+CC would need 2x the symbol rate). Encode-only
// onboard; the GROUND role strips parity without correction (the frame
// CRC rejects corrupted frames downstream).
#ifndef FSK_TM_RS
#define FSK_TM_RS 1
#endif

#ifndef FSK_CRC
// #define FSK_CRC LR20XX_RADIO_FSK_CRC_2_BYTES
#define FSK_CRC LR20XX_RADIO_FSK_CRC_OFF
#endif

#ifndef FSK_WHITENING
// Semtech whitening off: it is proprietary and a CCSDS ground station
// would not undo it.
#define FSK_WHITENING LR20XX_RADIO_FSK_WHITENING_OFF
#endif

// CCSDS pseudo-randomization (LFSR x^8+x^7+x^5+x^3+1, all-ones seed, reset
// per frame). Off in both directions per the ground segment configuration;
// must match the ground station. NOTE: with randomization and Semtech
// whitening both off, long constant-bit runs in frame data are sent as-is,
// which can degrade RX clock recovery over a 255-byte packet.
#ifndef FSK_TM_RANDOMIZE
#define FSK_TM_RANDOMIZE 0
#endif

#ifndef FSK_TC_RANDOMIZE
#define FSK_TC_RANDOMIZE 0
#endif

// Syncwords: the *_BITS least significant bits of a 64-bit value
// (MSB = syncword[0] MSB), transmitted most significant bit first.
// The chip has a single FSK syncword register, so the active value is
// written before each TX/RX operation.

// TM (downlink) channel: 32-bit CCSDS Attached Sync Marker (CCSDS 131.0-B).
// Over the air: preamble | ASM | fixed-length TM frame.
constexpr uint8_t FSK_TM_SYNCWORD[LR20XX_RADIO_FSK_SYNCWORD_LENGTH] = {
    0x00, 0x00, 0x00, 0x00, 0x1A, 0xCF, 0xFC, 0x1D};
constexpr uint8_t FSK_TM_SYNCWORD_BITS = 32;

// TC (uplink) channel: 16-bit CLTU Start Sequence (CCSDS 231.0-B).
// Over the air: PLOP idle (= preamble) | 0xEB90 | BCH codeblocks | tail.
// RX (spacecraft): the standard 16-bit CLTU Start Sequence, so a real
// CCSDS ground station is received as-is (RX path verified clean on HW).
constexpr uint8_t FSK_TC_RX_SYNCWORD[LR20XX_RADIO_FSK_SYNCWORD_LENGTH] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xEB, 0x90};
constexpr uint8_t FSK_TC_RX_SYNCWORD_BITS = 16;

// TX (LR2021 acting as ground, bench only): the LR2021 TX modem corrupts
// the first payload byte after a 16-bit syncword (replaced by the preamble
// value; verified against BCH parity on HW). Transmitting one PLOP idle
// octet inside the syncword (0x55EB90, 24 bits) avoids the quirk and
// matches what a real ground station puts on the air anyway.
constexpr uint8_t FSK_TC_TX_SYNCWORD[LR20XX_RADIO_FSK_SYNCWORD_LENGTH] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x55, 0xEB, 0x90};
constexpr uint8_t FSK_TC_TX_SYNCWORD_BITS = 24;

// BER-test channel (RadioBerTest command, bench only): a dedicated 32-bit
// syncword so the raw PRBS test packets never collide with the TM / TC CCSDS
// channels and the receiving radio only counts the test frames. The raw BER
// path (fskBerTx / fskBerRx) bypasses all CCSDS coding (no RS / CLTU /
// randomization) so the software bit-compare measures the true channel BER.
constexpr uint8_t FSK_BER_SYNCWORD[LR20XX_RADIO_FSK_SYNCWORD_LENGTH] = {
    0x00, 0x00, 0x00, 0x00, 0x93, 0x0B, 0x51, 0xDE};
constexpr uint8_t FSK_BER_SYNCWORD_BITS = 32;

// ---------------------------------------------------------------------------
// CLTU (CCSDS 231.0-B) parameters for the TC uplink
// ---------------------------------------------------------------------------

// Each BCH(63,56) codeblock carries 7 data bytes plus one parity octet
// (7 complemented parity bits + filler '0').
constexpr uint16_t CLTU_BLOCK_DATA = 7;
constexpr uint16_t CLTU_BLOCK_SIZE = 8;

// Tail Sequence closing a CLTU (one codeblock that always fails decode).
constexpr uint8_t CLTU_TAIL_SEQUENCE[CLTU_BLOCK_SIZE] = {
    0xC5, 0xC5, 0xC5, 0xC5, 0xC5, 0xC5, 0xC5, 0x79};

// Fill byte for the last codeblock and PLOP idle padding (alternating bits).
constexpr uint8_t CLTU_FILL_BYTE = 0x55;

// Leading idle pad octets transmitted before codeblock 0. Not needed since
// the TX-side first-byte corruption is avoided by the 24-bit TX syncword
// (see FSK_TC_TX_SYNCWORD); kept as a config point.
constexpr uint16_t CLTU_LEAD_PAD = 0;

// Maximum leading bytes the decoder skips while hunting for the first
// valid codeblock (defensive: absorbs pad bytes and timing uncertainty).
constexpr uint16_t CLTU_MAX_LEAD_SKIP = 4;

// Maximum codeblocks accepted per CLTU: 31 * 7 = 217 data bytes, which must
// cover the largest uplinked TC frame. CAUTION: the radio FIFO is 256 bytes
// and is written/read in a single transfer (no threshold streaming), so the
// TC packet (pad + blocks * 8 + 8 tail) must stay <= 256 (verified good at
// exactly 256 on HW). At 32 blocks (264 B) the transfer wraps and the idle
// padding overwrites codeblock 0.
#ifndef FSK_CLTU_MAX_BLOCKS
#define FSK_CLTU_MAX_BLOCKS 31
#endif

// Fixed over-the-air packet length of the TC channel: pad + codeblocks + tail.
constexpr uint16_t FSK_CLTU_PKT_LEN =
    CLTU_LEAD_PAD + FSK_CLTU_MAX_BLOCKS * CLTU_BLOCK_SIZE + CLTU_BLOCK_SIZE;

// TX operation timeout used when starting TX, in ms.
constexpr uint32_t FSK_TX_TIMEOUT_MS = 5000;

// RTC-step value selecting RX continuous mode (see lr20xx_radio_common.h).
constexpr uint32_t FSK_RX_CONTINUOUS = 0xFFFFFF;

// ---------------------------------------------------------------------------
// Listen-before-talk (half-duplex channel access)
//
// Every radio link is half-duplex, so a node must not key up while its peer
// is transmitting to it. Two independent gates run before each TX:
//
//  1. Link-aware carrier sense (always on): the radio already sits in RX, so
//     a PREAMBLE_DETECTED / SYNC_WORD_HEADER_VALID IRQ without RX_DONE means
//     a frame of OUR link is arriving. Only our own modulation / syncword
//     triggers it, so foreign users of a crowded band do not hold off TX.
//
//  2. Hardware CAD-LBT (configurable per radio): the LR2021 non-LoRa CAD
//     (SetCadParams exit_mode = TX) measures RSSI on the TX channel for the
//     listen window and only keys the PA if it stayed below the threshold;
//     otherwise CAD_DETECTED is raised and the chip falls back to standby.
//     This is plain energy detection: useful on the quiet S-band link, but
//     on 70 cm amateur UHF the spacecraft sees the whole footprint's
//     emitters and the ground sees the satellite near the noise floor, so
//     it is off by default for FSK. Calibrate thresholds against the
//     FskNoiseFloor / FlrcNoiseFloor telemetry before enabling.
//
// A busy channel defers the frame by a random backoff and retries; after
// LBT_MAX_ATTEMPTS busy results the frame is sent anyway (a saturated or
// mis-thresholded channel must never stall the link).
//
// Turnaround window: after each of its own transmissions a node keeps
// listening for turnaroundMs before it may key up again. Without it a node
// with a backlog sends back to back: its next frame starts within one run
// tick of TX_DONE, long before the peer has even seen the previous one end
// (the peer needs its RX latency + TX latency to react), so the peer's frame
// lands on top of the next one and both are lost (seen on the UHF bench with
// the SDR ground station: TM lost under every TC). If the peer's preamble /
// syncword arrives inside the window, link-aware sense holds TX until its
// frame has been received. Size it above the peer's reaction time; it costs
// downlink throughput only while there is a backlog.
// ---------------------------------------------------------------------------

// Channel-access attempts per frame before it is transmitted regardless.
#ifndef LBT_MAX_ATTEMPTS
#define LBT_MAX_ATTEMPTS 8
#endif

// FSK (UHF, 9600 bps) defaults. Backoff covers about one TM/TC packet
// (~220 ms on the air) so a deferred node lets the peer's burst finish.
#ifndef LBT_FSK_CAD_ENABLED
#define LBT_FSK_CAD_ENABLED false
#endif
#ifndef LBT_FSK_THRESHOLD_DBM
#define LBT_FSK_THRESHOLD_DBM (-100)
#endif
#ifndef LBT_FSK_LISTEN_US
#define LBT_FSK_LISTEN_US 5000  // ~48 bits at 9600 bps
#endif
#ifndef LBT_FSK_BACKOFF_MIN_MS
#define LBT_FSK_BACKOFF_MIN_MS 50
#endif
#ifndef LBT_FSK_BACKOFF_MAX_MS
#define LBT_FSK_BACKOFF_MAX_MS 400
#endif
#ifndef LBT_FSK_TURNAROUND_MS
// The SDR ground station's reaction time (USB RX + TX buffering) is tens of
// ms to ~150 ms; measure it with the ground modem's loopback-latency log.
#define LBT_FSK_TURNAROUND_MS 200
#endif

// FLRC (S-band) defaults. Packets last a few ms, so the backoff is short.
#ifndef LBT_FLRC_CAD_ENABLED
#define LBT_FLRC_CAD_ENABLED true
#endif
#ifndef LBT_FLRC_THRESHOLD_DBM
#define LBT_FLRC_THRESHOLD_DBM (-90)
#endif
#ifndef LBT_FLRC_LISTEN_US
#define LBT_FLRC_LISTEN_US 500
#endif
#ifndef LBT_FLRC_BACKOFF_MIN_MS
#define LBT_FLRC_BACKOFF_MIN_MS 5
#endif
#ifndef LBT_FLRC_BACKOFF_MAX_MS
#define LBT_FLRC_BACKOFF_MAX_MS 50
#endif
#ifndef LBT_FLRC_TURNAROUND_MS
// S-band runs split TX/RX frequencies: the peer's uplink does not collide
// with our downlink, so no gap is needed.
#define LBT_FLRC_TURNAROUND_MS 0
#endif

// Longest a peer frame can occupy the channel once its syncword is seen
// (link-aware hold, released early by RX_DONE / an RX error). FSK: the
// largest fixed packet (256 B) at the FSK bit rate, plus margin. FLRC: the
// largest payload at the raw rate with rate-1/2 coding, plus margin.
constexpr uint32_t LBT_FSK_FRAME_HOLD_US =
    static_cast<uint32_t>((8ULL * 256ULL * 1000000ULL) / FSK_BITRATE_BPS) + 20000u;
constexpr uint32_t LBT_FLRC_FRAME_HOLD_US =
    static_cast<uint32_t>((2ULL * 8ULL * LR2021::LR2021Manager::FLRC_MAX_PAYLOAD * 1000000ULL) /
                          FLRC_RAW_BITRATE_BPS) + 2000u;

// Hold after a preamble detection that has not (yet) produced a syncword:
// preamble + syncword airtime, plus margin. A false preamble detection on
// noise therefore costs at most this long.
constexpr uint32_t LBT_FSK_PREAMBLE_HOLD_US =
    static_cast<uint32_t>((2ULL * (FSK_PREAMBLE_BITS + 32ULL) * 1000000ULL) / FSK_BITRATE_BPS);
constexpr uint32_t LBT_FLRC_PREAMBLE_HOLD_US = 1000u;

// Idle-channel RSSI telemetry (FskNoiseFloor / FlrcNoiseFloor): sampled every
// LBT_NOISE_SAMPLE_MS while the radio idles in RX; the minimum over each
// LBT_NOISE_REPORT_MS window is reported (bursts of traffic are excluded).
#ifndef LBT_NOISE_SAMPLE_MS
#define LBT_NOISE_SAMPLE_MS 100
#endif
#ifndef LBT_NOISE_REPORT_MS
#define LBT_NOISE_REPORT_MS 2000
#endif

#endif /* _LR2021_LR2021CFG_HPP_ */
