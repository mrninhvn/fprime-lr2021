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

// Syncword #1, used for both TX and RX matching.
constexpr uint8_t FLRC_SYNCWORD[LR20XX_RADIO_FLRC_SYNCWORD_LENGTH] = { 0x90, 0x56, 0x34, 0x12 };

// TX/RX operation timeout used when starting TX, in ms.
constexpr uint32_t FLRC_TX_TIMEOUT_MS = 5000;

// RTC-step value selecting RX continuous mode (see lr20xx_radio_common.h).
constexpr uint32_t FLRC_RX_CONTINUOUS = 0xFFFFFF;

#endif /* _LR2021_LR2021CFG_HPP_ */
