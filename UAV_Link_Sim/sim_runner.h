#pragma once

#include <string>
#include <vector>
#include <complex>
#include <sstream>

#include "common_defs.h"

enum class RunMode {
    LOOPBACK,
    AWGN,
    USRP
};

struct TestResult {
    size_t total_compared_bits = 0;
    size_t total_bit_errors = 0;
    size_t decoded_frames = 0;
    double total_ber = 0.0;

    // 文件传输时可用
    bool file_saved = false;
    std::string saved_file_path;

    std::string log_text;
};

struct SweepPoint {
    double snr_db = 0.0;
    double ber = 0.0;
    size_t decoded_frames = 0;
};

struct SweepResult {
    std::vector<SweepPoint> points;
    std::string log_text;
};

SweepResult run_awgn_sweep(
    double snr_start,
    double snr_end,
    double snr_step,
    int tx_repeat_frames,
    double center_freq_hz,
    ModulationType modulation
);

std::string mode_to_string(RunMode mode);
std::string modulation_to_string(ModulationType m);

ModulationType parse_modulation(const std::string& s);
RunMode parse_mode(const std::string& s);

// =========================
// 原有随机 bit 测试：保留
// =========================
TestResult run_one_test(
    RunMode mode,
    double awgn_snr_db,
    int tx_repeat_frames,
    double center_freq_hz,
    ModulationType modulation
);

// =========================
// 新增：文件传输测试
// input_file_path: 要发送的 txt/jpg/mp4 等文件
// output_file_path: 接收端恢复后保存到哪里
// =========================
TestResult run_file_transfer_test(
    RunMode mode,
    double awgn_snr_db,
    double center_freq_hz,
    ModulationType modulation,
    const std::string& input_file_path,
    const std::string& output_file_path
);