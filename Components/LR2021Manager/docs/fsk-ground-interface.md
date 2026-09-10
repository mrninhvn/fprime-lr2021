# FSK Channel Coding Chain — Ground Software Reference

This document is a **byte-exact, implementation-facing** reference for whoever
writes or verifies the ground-station software (SDR modem, GNU Radio flowgraph,
or any non-F´ ground implementation) that talks to the `LR2021Manager` FSK/GMSK
link. It is deliberately independent of F´/Zephyr internals — everything below is
either an unmodified CCSDS algorithm or a project-specific parameter needed to
interoperate with the onboard radio.

For the F´ component architecture (ports, commands, telemetry) see the
[LR2021Manager SDD](sdd.md); for the ground *relay board*'s own FLRC link and APID
routing (a separate, non-CCSDS-coded link) see
[`ground-channel-coding.md`](../../../fprime-vspc-com/VspcCom/docs/ground-channel-coding.md).

## Scope

The FSK mode implements a two-channel CCSDS UHF link:

| Channel | Direction | Standard | Coding |
|---|---|---|---|
| TM (telemetry) | Spacecraft → ground | CCSDS 131.0-B / 132.0-B | Reed–Solomon (255,223), interleave depth 1 |
| TC (telecommand) | Ground → spacecraft | CCSDS 231.0-B / 232.0-B | BCH(63,56) CLTU |

Both channels share the same GMSK modulator; only the syncword, coding, and packet
length differ. Everything downstream of "bits off the radio" (frame CRC, Space
Packet contents, command dispatch) is standard CCSDS/F´ and out of scope here — this
document stops at "recovered TC/TM transfer frame bytes."

## Radio / modulation parameters

| Parameter | Value | Notes |
|---|---|---|
| Modulation | GMSK | Gaussian-filtered 2-FSK, continuous phase |
| Bit rate | 9600 sym/s | `FSK_BITRATE_BPS`; ground segment cap is 10 kbps |
| Modulation index | h = 0.5 | Fixed relationship: `fdev = bitrate / 4 = 2400 Hz` |
| Gaussian filter BT | 0.5 | Per CCSDS 211.1 (Proximity-1) GMSK profile |
| Coding (line) | NRZ-L, non-precoded | No differential/NRZ-M encoding |
| Preamble | 32 bits, alternating `01010101…` | Same for TM and TC |
| Whitening (Semtech) | Off | Proprietary to the LR2021; a CCSDS ground station must not apply/undo it |
| CCSDS pseudo-randomization | **Off**, both directions | See "Randomizer" below — implemented but disabled; must match if re-enabled |
| Doppler @ 437 MHz, 550 km LEO | ±10.2 kHz uncompensated, ≤150 Hz/s | Assume the ground station compensates (TLE-based); RX filter margin sized for a compensated link |

## TM downlink (spacecraft → ground)

### Chain, spacecraft side (encode)

```
Space Packets (CCSDS 133.0-B)
  -> TM transfer frame: 6 B header + data field + idle fill + 2 B FECF (CRC-16)
     fixed length 223 bytes                              [= RS data field]
  -> Reed-Solomon (255,223) encode -> +32 B parity
  -> [pseudo-randomizer -- DISABLED]
  -> GMSK modulate, preceded by preamble + ASM
```

### On-air packet layout

| Field | Length | Value |
|---|---|---|
| Preamble | 32 bit | `0101…` |
| ASM (Attached Sync Marker) | 4 B | `1A CF FC 1D` (CCSDS standard ASM) |
| TM transfer frame | 223 B | 6 B header, data field + idle fill, 2 B FECF (CRC-16) |
| RS parity | 32 B | See below |

Total packet after the ASM: **255 bytes** (223 + 32), transmitted at 9600 sym/s
(≈ 212 ms on air for the coded block, ~219 ms including preamble+ASM).

### Reed–Solomon (255,223), CCSDS 131.0-B

This is the **standard CCSDS RS code**, not a custom variant — a compliant ground
station's existing RS(255,223) decoder works unmodified. Parameters, for
cross-checking an implementation:

- **Field**: GF(256), generator polynomial `F(x) = x^8 + x^7 + x^2 + x + 1` (hex `0x187`).
- **Code generator polynomial**: `g(x) = ∏_{j=112}^{143} (x − (α^11)^j)`, where `α` is
  a root of `F(x)` (primitive element). Degree 32 (32 parity symbols), interleave
  depth **I = 1** (no interleaving).
- **Symbol basis**: CCSDS 131.0-B represents RS symbols on the wire in the **dual
  (Berlekamp) basis**, not the natural polynomial basis. The encoder here works
  internally in the conventional (polynomial) basis and converts to/from dual basis
  at the data boundary (see `LR2021Rs.cpp` for the exact basis-conversion
  construction via the trace function `Tr(x) = x + x² + x⁴ + … + x¹²⁸`). **A ground
  RS decoder must also operate on dual-basis symbols** — this is standard CCSDS
  behavior, so any CCSDS-compliant RS(255,223) implementation (e.g. GNU Radio's
  `ccsds_rs_decoder`, or a from-scratch implementation per 131.0-B Annex) already
  does this; it is called out here only because it is easy to get wrong if
  implementing RS from a generic (non-CCSDS) reference.
- **Systematic encoding**: the 223 data bytes are transmitted unchanged, followed by
  32 parity bytes (highest-degree remainder coefficient first).
- **Decoding**: the onboard encoder is transmit-only. **The ground station is
  expected to perform full RS decoding** (correcting up to 16 byte errors per
  codeword) — this is exactly what makes the link viable at 9600 sym/s with margin.
  The satellite side (when configured in the `GROUND` bench role) only strips the
  32 parity bytes without correction, relying on the frame's own CRC-16 to reject
  corrupted frames; do not model that shortcut as the expected ground behavior.

### Randomizer (disabled, but implemented — for reference / future use)

If `FSK_TM_RANDOMIZE` is ever turned on, the CCSDS pseudo-randomizer is applied to
**everything after the ASM** (i.e. the 223 B frame + 32 B RS parity, all 255 bytes),
reset to the all-ones seed at the start of each frame:

- LFSR polynomial: `x^8 + x^7 + x^5 + x^3 + 1`
- Seed: `0xFF` (all ones) at the start of every frame
- Output sequence (first bytes, MSB-first per the standard): `FF 48 0E C0 9A 0D 70 BC …`
- Self-inverse: XOR-ing the sequence a second time recovers the original data —
  the same function randomizes and derandomizes.

**Currently disabled in both directions** — a ground implementation should NOT
apply/remove this by default; only do so if the onboard config (`FSK_TM_RANDOMIZE`)
is changed to `1`, and it must be changed to `1` on both ends simultaneously.

## TC uplink (ground → spacecraft)

### Chain, ground side (encode) — what the ground software must produce

```
TC transfer frame (CCSDS 232.0-B, Type-BD)
  5 B header (SCID 0x0044 + VC ID + frame length) + data + 2 B FECF (CRC-16)
  variable length, <= 217 bytes
  -> [pseudo-randomizer -- DISABLED]
  -> split into 7-byte data groups, BCH(63,56) encode each -> 8-byte codeblocks
  -> append Tail Sequence codeblock
  -> idle-pad to the fixed 256-byte capture window
  -> GMSK modulate, preceded by PLOP idle sequence + CLTU start sequence
```

### On-air packet layout (as accepted by the onboard receiver)

| Field | Length | Value |
|---|---|---|
| PLOP idle / preamble | ≥ 16 bit | Alternating `0101…` (radio preamble = PLOP idle) |
| CLTU Start Sequence | 2 B | `EB 90` (standard CCSDS 231.0-B start sequence — **use this**, not the onboard TX quirk value below) |
| BCH codeblocks | n × 8 B | Up to 31 blocks (≤ 217 data bytes); see below |
| Tail Sequence | 8 B | `C5 C5 C5 C5 C5 C5 C5 79` (fixed, standard) |
| Idle padding | to 256 B total | Fill byte `0x55`, alternating bits |

The receiver's capture window is a **fixed 256 bytes** (a LR2021 FIFO constraint,
not a protocol requirement): `n_blocks * 8 + 8 (tail) + idle_pad = 256`, so
`n_blocks` up to 31 (217 data bytes) with the remainder idle-padded. **A real ground
station transmitting a variable-length CLTU is fully compatible** — the onboard
receiver stops decoding at the tail sequence (or the first bad codeblock) and
discards anything after it, so a shorter CLTU followed by idle/noise up to 256 B
works correctly. There is no need for the ground station to pad to 256 B itself.

> **Ground TX syncword note (asymmetry, receive-only concern).** The onboard radio
> chip has a receive-path quirk unrelated to a compliant ground station: its own
> **transmit** modem corrupts the first payload byte after a 16-bit syncword. When
> the *onboard* radio is configured to transmit CLTUs (bench `GROUND` role only), it
> uses a non-standard 24-bit TX syncword (`55 EB 90`) to work around this. This does
> **not** apply to a real ground station transmitting to the spacecraft: the
> spacecraft's **receiver** matches the standard 16-bit `EB 90` start sequence
> exactly as CCSDS 231.0-B specifies, and is verified clean on hardware for that
> case. A ground implementation should just transmit the standard `EB90` start
> sequence — no special-casing needed.

### BCH(63,56) codeblock, CCSDS 231.0-B

Each codeblock carries 7 data bytes (56 bits) + 1 parity octet (8 bits) = 8 bytes
(64 bits transmitted, of which 63 are the BCH codeword and the leading bit implicit
per the standard's convention — see below for the exact bit layout used here).

- **Generator polynomial**: `g(x) = x^7 + x^6 + x^2 + 1`
- **Encoding**: compute the BCH parity of the 56 data bits (7 bytes, MSB-first,
  most-significant byte/bit transmitted first) by polynomial division (LFSR) over
  GF(2) using `g(x)`.
- **Parity octet bit layout**: the 7 remainder bits from the division are
  **complemented** (bitwise NOT) and placed in bits 7 down to 1 of the parity
  octet; **bit 0 (LSB) is a fixed filler bit set to `0`**.
- **Reference implementation** (equivalent LFSR form, for cross-checking against an
  independent implementation): a 7-bit shift register `sr`, initialized to 0;
  for each of the 56 input bits (MSB-first across the 7 data bytes):
  `feedback = (sr>>6) XOR data_bit; sr = (sr<<1) & 0x7F; if feedback: sr ^= 0x45`
  (`0x45` = `100 0101b` = `x^6 + x^2 + 1`, i.e. `g(x)` without its `x^7` leading
  term). After all 56 bits: `parity_octet = (~sr << 1) & 0xFE` (complement, shift
  left one to leave the filler bit 0 in bit 0).
- **Verification**: this construction was checked byte-exactly on hardware against
  the CCSDS 231.0-B BCH parity definition during bring-up; treat it as ground truth
  for interoperability testing.

### Codeblock decode / error detection (what the ground decoder does symmetrically, and what the spacecraft does on receive)

A codeblock is valid iff re-computing its parity octet from the 7 data bytes
reproduces the received parity octet exactly (single-error-detecting, per CCSDS
231.0-B's BCH shortened code — it is **not** a correcting code in this
implementation; a codeblock with a bad parity is rejected outright, matching the
onboard spacecraft receiver's own behavior — a spec-compliant ground receiver
should do the same when receiving CLTUs on a bench-role downlink test).

The spacecraft's decoder additionally tolerates a few bytes of leading garbage
(defensive skip, up to 4 bytes) before the first codeblock, to absorb sync/timing
uncertainty — this is a defensive measure, not something a ground TX needs to
produce or compensate for.

### Randomizer (disabled, but implemented — for reference / future use)

Same LFSR as the TM channel (`x^8+x^7+x^5+x^3+1`, seed `0xFF` per frame), applied
(if ever enabled via `FSK_TC_RANDOMIZE`) to the **data + fill octet stream before
BCH encoding** (i.e. before the codeblocks are formed; the BCH parity octets
themselves are never randomized). **Currently disabled** — do not apply by default.

## Quick byte-layout summary (both channels, for a protocol analyzer)

```
TM packet (255 B on air, after preamble+ASM):
  [ 223 B TM transfer frame ][ 32 B RS parity ]

TC packet (up to 256 B on air, after preamble+start-sequence):
  [ 8 B codeblock ] x n  [ 8 B tail = C5 C5 C5 C5 C5 C5 C5 79 ]  [ idle 0x55 fill to 256 B ]
     each codeblock = [ 7 B data ][ 1 B BCH parity ]
```

## Things that will NOT interoperate if changed independently

These are coupled parameters — changing one without the other breaks the link
end-to-end. Ground software should treat them as a fixed pair/set matching the
onboard config (`LR2021Cfg.hpp`) currently in use:

- Bit rate and frequency deviation (`h = 0.5` relationship)
- RS on/off, and — if on — the fixed 223-byte TM frame length (`FSK_FIXED_PAYLOAD_LEN`
  must equal the RS data-field size, 223)
- Randomizer on/off, independently per channel, but must match on both ends of that
  channel
- TC/TM syncword values (`EB90` TC start sequence, `1ACFFC1D` TM ASM)
- Maximum CLTU codeblock count (31) bounding the largest TC frame (217 B) accepted
