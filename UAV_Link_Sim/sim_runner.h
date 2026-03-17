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

TestResult run_one_test(
    RunMode mode,
    double awgn_snr_db,
    int tx_repeat_frames,
    double center_freq_hz,
    ModulationType modulation
);