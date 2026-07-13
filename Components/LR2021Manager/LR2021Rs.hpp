// ======================================================================
// \title  LR2021Rs.hpp
// \author ninhdh4
// \brief  CCSDS Reed-Solomon (255,223) encoder (CCSDS 131.0-B)
//
// Systematic RS over GF(256) with field polynomial x^8+x^7+x^2+x+1,
// code generator roots (alpha^11)^112 .. (alpha^11)^143, interleave
// depth I = 1. Symbols are represented in the dual (Berlekamp) basis
// as the standard requires. Encode-only: decoding is the ground
// station's job; onboard the frame CRC rejects corrupted frames.
// ======================================================================

#ifndef LR2021_LR2021RS_HPP
#define LR2021_LR2021RS_HPP

#include <Fw/Types/BasicTypes.hpp>

namespace LR2021 {
namespace Rs {

constexpr U16 RS_DATA = 223;    //!< Data bytes per codeword (the TM frame)
constexpr U16 RS_PARITY = 32;   //!< Parity bytes appended
constexpr U16 RS_BLOCK = 255;   //!< Codeword length on the air

//! Append the 32 RS parity bytes for 223 data bytes.
//! \p data and \p parity are in the dual-basis representation used on the
//! wire (i.e. exactly the bytes before/after the frame in the codeblock).
void encode(const U8* data, U8* parity);

}  // namespace Rs
}  // namespace LR2021

#endif
