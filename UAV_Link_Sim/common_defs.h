#pragma once

#include <vector>
#include <complex>
#include <string>
#include <cmath>

// ====================== 修复1：跨平台PI常量定义 ======================
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
constexpr double PI = M_PI; // 统一全局PI常量，兼容原有代码的PI/M_PI两种写法

// ====================== 修复2：兼容C++11的clamp模板函数 ======================
template <typename T>
inline T clamp_value(T x, T lo, T hi) {
    return (x < lo) ? lo : (x > hi ? hi : x);
}

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

// 信源模式枚举
enum class SourceMode {
    RandomBits = 0,
    FileBits = 1
};

// 发射端配置结构体
struct TransmitterConfig {
    // 可选参数
    bool enable_cfo = true;                          // 是否启用接收端 CFO 估计与补偿
    bool connect = false;                            // 是否连接 USRP
    double Rb = 50e3;                                // 信息速率 (4K~200K)
    int hop_pattern = 1;
    FunctionType function = FunctionType::RemoteControl;
    ModulationType modulation = ModulationType::FM;

    // 信源模式
    SourceMode source_mode = SourceMode::RandomBits;

    // 文件传输模式下使用
    std::string input_file_path;
    VecInt file_bits;
    size_t file_bit_offset = 0;

    // 默认参数
    int samp = 8;                                    // 上采样率
    double spsp = 32.0 / 5.0;                        // 扩频倍数
    double RsRate = 31.0 / 15.0;                     // 信道编码率 (31/15)
    int Pulse_num = 31;                              // 消息脉冲数
    int coarse_length = 4;                           // 粗同步序列长度
    int fine_length = 1;                             // 精同步序列长度
    int n = 10;                                      // 帧扩展倍数
    int frame_bit = 75;                              // 一帧比特数

    // ZP：帧末尾零填充长度（按符号数）
    int zp_sym = 33;

    // 计算衍生参数
    double FrameRate = (coarse_length + fine_length + Pulse_num * n) / (Pulse_num * n);
    double fs = 0;                                   // 采样率（后续计算）

    // CCSK序列
    VecInt ccskcode = {
        1,0,1,1,1,0,1,0,0,0,1,1,1,1,0,1,
        0,0,1,0,0,0,0,0,0,1,1,0,0,1,1,0
    };
};