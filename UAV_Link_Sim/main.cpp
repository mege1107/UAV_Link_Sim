#include <iostream>
#include <iomanip>

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

        ChannelConfig ch;
        ch.enable_awgn = true;
        ch.snr_dB = snr_db;
        ch.seed = 123;

        ch.enable_sto = true;
        ch.sto_samp = 0;

        ch.enable_cfo = true;
        ch.cfo_hz = 0;

        ch.enable_sfo = true;
        ch.sfo_ppm = 2.0;

        auto result = run_channel_test(
            snr_db,
            tx_frames,
            fc,
            mod,
            rate,
            hop,
            ch
        );

        std::cout << std::fixed << std::setprecision(3)
            << "STO=" << ch.sto_samp << " samp"
            << " | CFO=" << ch.cfo_hz << " Hz"
            << " | SFO=" << ch.sfo_ppm << " ppm"
            << " | Frames=" << tx_frames
            << " | BER=" << std::scientific << std::setprecision(6) << result.total_ber
            << std::fixed << " | bit_errors=" << result.total_bit_errors
            << " | compared_bits=" << result.total_compared_bits
            << " | decoded_frames=" << result.decoded_frames << "\n";
    }
    catch (const std::exception& e)
    {
        std::cerr << "ERROR: " << e.what() << std::endl;
    }

    return 0;
}
