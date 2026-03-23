#include <iostream>
#include <iomanip>
#include <string>
#include <vector>

#include "sim_runner.h"
#include "channel.h"

int main()
{
    try
    {
        std::cout << "============================\n";
        std::cout << "  CUSTOM CHANNEL TEST\n";
        std::cout << "============================\n";

        // ===== 基本参数 =====
        double snr_db = 20.0;
        int tx_frames = 10;
        double fc = 2.45e9;
        double rate = 50000;
        int hop = 1;

        ModulationType mod = ModulationType::QPSK;

        // ===== main里只改这几组数组即可做单因素 BER 测试 =====
        const std::vector<int> sto_cases = { 0, 4, 8, 12, 16, 20 };
        const std::vector<double> cfo_cases = { 0.0, 500.0, 1000.0, 2000.0, 4000.0, 6000.0 };
        const std::vector<double> sfo_cases = { 0.0, 5.0, 10.0, 20.0, 40.0, 80.0 };

        auto make_base_channel = [snr_db]() {
            ChannelConfig ch;
            ch.enable_awgn = true;
            ch.snr_dB = snr_db;
            ch.seed = 123;
            return ch;
        };

        auto print_result = [](const std::string& factor_name,
            double value,
            const char* unit,
            const TestResult& result) {
                std::cout << std::fixed << std::setprecision(3)
                    << factor_name << "=" << value << unit
                    << " | BER=" << std::scientific << std::setprecision(6) << result.total_ber
                    << std::fixed << " | bit_errors=" << result.total_bit_errors
                    << " | compared_bits=" << result.total_compared_bits
                    << " | decoded_frames=" << result.decoded_frames << "\n";
        };

        auto run_sweep = [&](const std::string& title,
            const std::string& factor_name,
            const std::vector<double>& values,
            const auto& config_setter) {
                std::cout << "\n========== " << title << " ==========\n";
                for (double value : values) {
                    ChannelConfig ch = make_base_channel();
                    config_setter(ch, value);

                    auto result = run_channel_test(
                        snr_db,
                        tx_frames,
                        fc,
                        mod,
                        rate,
                        hop,
                        ch
                    );

                    const char* unit = "";
                    if (factor_name == "STO") unit = " samp";
                    else if (factor_name == "CFO") unit = " Hz";
                    else if (factor_name == "SFO") unit = " ppm";

                    print_result(factor_name, value, unit, result);
                }
        };

        std::vector<double> sto_values;
        sto_values.reserve(sto_cases.size());
        for (int v : sto_cases) {
            sto_values.push_back(static_cast<double>(v));
        }

        run_sweep("STO SWEEP", "STO", sto_values,
            [](ChannelConfig& ch, double value) {
                ch.enable_sto = true;
                ch.sto_samp = static_cast<int>(value);
            });

        run_sweep("CFO SWEEP", "CFO", cfo_cases,
            [](ChannelConfig& ch, double value) {
                ch.enable_cfo = true;
                ch.cfo_hz = value;
            });

        run_sweep("SFO SWEEP", "SFO", sfo_cases,
            [](ChannelConfig& ch, double value) {
                ch.enable_sfo = true;
                ch.sfo_ppm = value;
            });
    }
    catch (const std::exception& e)
    {
        std::cerr << "ERROR: " << e.what() << std::endl;
    }

    return 0;
}
