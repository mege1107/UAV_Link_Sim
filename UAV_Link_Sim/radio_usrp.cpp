#pragma once
#include "radio_if.h"

#ifdef WITH_UHD
#include "usrp_driver.h"
#endif

class USRPRadio : public IRadio {
public:
#ifdef WITH_UHD
    explicit USRPRadio(const USRPDriver::Config& cfg) : usrp_(cfg) {
        usrp_.init();
    }

    void start_rx() override { usrp_.start_rx_now(); }
    void stop_rx() override { usrp_.stop_rx(); }

    size_t send_burst(const VecComplex& samples) override {
        return usrp_.send_burst(samples);
    }

    size_t recv_samples(size_t nsamps, VecComplex& out) override {
        return usrp_.recv_samples(nsamps, out);
    }
private:
    USRPDriver usrp_;
#else
    // 没有UHD也能编译：给一个占位实现
    explicit USRPRadio(...) {}
    size_t send_burst(const VecComplex&) override { return 0; }
    size_t recv_samples(size_t, VecComplex&) override { return 0; }
#endif
};