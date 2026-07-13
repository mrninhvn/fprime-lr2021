// ======================================================================
// \title  LR2021Rs.cpp
// \author ninhdh4
// \brief  CCSDS Reed-Solomon (255,223) encoder (CCSDS 131.0-B)
//
// All tables are derived at first use from the standard's definitions:
//  - GF(256) with F(x) = x^8 + x^7 + x^2 + x + 1
//  - generator polynomial g(x) = prod_{j=112..143} (x - (alpha^11)^j)
//  - dual (Berlekamp) basis {l_i} defined by trace-duality with the
//    polynomial basis: Tr(l_i * alpha^j) = delta_ij, bit l_0 = MSB.
// The wire format uses dual-basis symbols; encoding runs in the
// conventional basis with table conversion at the boundaries.
// ======================================================================

#include "Components/LR2021Manager/LR2021Rs.hpp"

#include <cstring>

namespace LR2021 {
namespace Rs {

namespace {

U8 gf_exp[510];       // antilog table, doubled to avoid modulo in gmul
U8 gf_log[256];
U8 genpoly[RS_PARITY];  // g(x) coefficients x^0..x^31 (x^32 is monic)
U8 to_conv[256];      // dual basis  -> conventional (polynomial) basis
U8 to_dual[256];      // conventional -> dual basis
bool inited = false;

U8 gmul(U8 a, U8 b) {
    if ((a == 0) || (b == 0)) {
        return 0;
    }
    return gf_exp[gf_log[a] + gf_log[b]];
}

// Trace over GF(2): Tr(x) = x + x^2 + x^4 + ... + x^128 (result 0 or 1)
U8 gtrace(U8 x) {
    U8 t = 0;
    for (U8 k = 0; k < 8; k++) {
        t ^= x;
        x = gmul(x, x);
    }
    return t;
}

void init() {
    if (inited) {
        return;
    }

    // GF(256) log/antilog for F(x) = x^8 + x^7 + x^2 + x + 1 (0x187)
    U16 x = 1;
    for (U16 i = 0; i < 255; i++) {
        gf_exp[i] = static_cast<U8>(x);
        gf_log[x] = static_cast<U8>(i);
        x <<= 1;
        if ((x & 0x100) != 0) {
            x ^= 0x187;
        }
    }
    for (U16 i = 255; i < 510; i++) {
        gf_exp[i] = gf_exp[i - 255];
    }

    // Basis conversion: an element y = sum z_i * l_i has dual coordinates
    // z_i = Tr(alpha^i * y); z_0 is the MSB (first bit transmitted).
    for (U16 y = 0; y < 256; y++) {
        U8 z = 0;
        for (U8 i = 0; i < 8; i++) {
            if (gtrace(gmul(gf_exp[i], static_cast<U8>(y))) != 0) {
                z |= static_cast<U8>(0x80 >> i);
            }
        }
        to_dual[y] = z;
    }
    for (U16 y = 0; y < 256; y++) {
        to_conv[to_dual[y]] = static_cast<U8>(y);
    }

    // g(x) = prod_{j=112}^{143} (x - (alpha^11)^j), computed iteratively
    U8 g[RS_PARITY + 1] = {0};
    g[0] = 1;
    for (U16 r = 0; r < RS_PARITY; r++) {
        const U8 root = gf_exp[(11 * (112 + r)) % 255];
        for (U16 k = r + 1; k > 0; k--) {
            g[k] = g[k - 1] ^ gmul(root, g[k]);
        }
        g[0] = gmul(root, g[0]);
    }
    memcpy(genpoly, g, RS_PARITY);  // g[RS_PARITY] == 1 (monic), implied

    inited = true;
}

}  // namespace

void encode(const U8* data, U8* parity) {
    init();

    // Systematic LFSR division in the conventional basis; data[0] is the
    // highest-degree message coefficient (first byte transmitted).
    U8 reg[RS_PARITY] = {0};
    for (U16 i = 0; i < RS_DATA; i++) {
        const U8 d = to_conv[data[i]];
        const U8 fb = d ^ reg[RS_PARITY - 1];
        for (U16 j = RS_PARITY - 1; j > 0; j--) {
            reg[j] = reg[j - 1] ^ gmul(fb, genpoly[j]);
        }
        reg[0] = gmul(fb, genpoly[0]);
    }

    // Remainder appended highest-degree coefficient first, back in the
    // dual-basis wire representation.
    for (U16 j = 0; j < RS_PARITY; j++) {
        parity[j] = to_dual[reg[RS_PARITY - 1 - j]];
    }
}

}  // namespace Rs
}  // namespace LR2021
