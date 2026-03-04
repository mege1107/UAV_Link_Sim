#pragma once
#include "common_defs.h"

class Receiver {
public:
    explicit Receiver(const TransmitterConfig& config);

    VecInt receive(const VecComplex& rx_signal);

private:
    TransmitterConfig config_;

    VecInt demodulateBPSK(const VecComplex& rx);
    VecInt despreadCCSK(const VecInt& chips);
};