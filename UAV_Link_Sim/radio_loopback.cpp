#include "radio_if.h"

class LoopbackRadio : public IRadio {
public:
    size_t send_burst(const VecComplex& samples) override {
        buf_ = samples;
        return samples.size();
    }

    size_t recv_samples(size_t nsamps, VecComplex& out) override {
        if (nsamps >= buf_.size()) out = buf_;
        else out.assign(buf_.begin(), buf_.begin() + nsamps);
        return out.size();
    }

private:
    VecComplex buf_;
};
