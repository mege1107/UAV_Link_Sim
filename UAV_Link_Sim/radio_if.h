#pragma once
#include "common_defs.h"
#include <cstddef>

class IRadio {
public:
    virtual ~IRadio() = default;

    // 发送一段基带采样（burst）
    virtual size_t send_burst(const VecComplex& samples) = 0;

    // 接收指定数量采样（nsamps）
    virtual size_t recv_samples(size_t nsamps, VecComplex& out) = 0;

    // 可选：启动/停止连续接收（USRP 用得上，仿真可为空实现）
    virtual void start_rx() {}
    virtual void stop_rx() {}
};