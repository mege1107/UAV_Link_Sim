#include "radio_if.h"
#include "channel.h"

class AWGNRadio : public IRadio {
public:
    explicit AWGNRadio(double snr_db) : snr_db_(snr_db) {}

    size_t send_burst(const VecComplex& samples) override {
        last_tx_ = samples;
        return samples.size();
    }

    size_t recv_samples(size_t nsamps, VecComplex& out) override {
        // 对 last_tx_ 加 AWGN，模拟“空中接收”
        VecComplex rx = Channel::awgn(last_tx_, snr_db_);
        // 模拟接收端只取前 nsamps（或全部）
        if (nsamps >= rx.size()) out = rx;
        else out.assign(rx.begin(), rx.begin() + nsamps);
        return out.size();
    }

private:
    double snr_db_;
    VecComplex last_tx_;
};