#pragma once
#include "common_defs.h"

class Channel {
public:
    static VecComplex awgn(const VecComplex& signal,
        double snr_dB);
};