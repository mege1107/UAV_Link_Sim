#pragma once

#include <vector>
#include <complex>
#include <string>

// 类型别名
using Complex = std::complex<double>;
using VecComplex = std::vector<Complex>;
using VecInt = std::vector<int>;
using VecDouble = std::vector<double>;

// 功能类型枚举
enum class FunctionType {
    RemoteControl,  // 遥控
    Telemetry       // 遥测
};

// 调制方式枚举
enum class ModulationType {
    QPSK,
    FSK,
    OOK,
    BPSK,
    QAM,
    FM,
    MSK
};

// 发射端配置结构体
struct TransmitterConfig {
    // 可选参数
    bool connect = false;                  // 是否连接USRP
    double Rb = 50e3;                      // 信息速率 (4K~200K)
    FunctionType function = FunctionType::RemoteControl;  // 功能类型
    ModulationType modulation = ModulationType::FM;        // 调制方式

    // 默认参数
    int samp = 8;                           // 上采样率
    double spsp = 32.0 / 5.0;              // 扩频倍数
    double RsRate = 31.0 / 15.0;           // 信道编码率 (31/15)
    int Pulse_num = 31;                     // 消息脉冲数
    int coarse_length = 4;                  // 粗同步序列长度
    int fine_length = 1;                    // 精同步序列长度
    int n = 10;                             // 帧扩展倍数
    int frame_bit = 75;                     // 一帧比特数

    //  ZP：帧末尾零填充长度（按“符号数”给；最终会乘 samp 变成“采样数”）
    // 例如 zp_sym=33, samp=8 => ZP = 264 个复采样
    int zp_sym = 33;

    // 计算衍生参数
    double FrameRate = (coarse_length + fine_length + Pulse_num * n) / (Pulse_num * n);
    double fs = 0;                          // 采样率（需后续计算）

    // 优化后的CCSK序列 (Link16关键技术)
    VecInt ccskcode = { 1,0,1,1,1,0,1,0,0,0,1,1,1,1,0,1,0,0,1,0,0,0,0,0,0,1,1,0,0,1,1,0 };
};