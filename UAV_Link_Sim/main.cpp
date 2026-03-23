#include <iostream>
#include <iomanip>
#include <vector>

#include "sim_runner.h"
#include "channel.h"

int main()
{
    try
    {
        std::cout << "========================================\n";
        std::cout << "  CUSTOM CHANNEL COMBINATION SWEEP\n";
        std::cout << "========================================\n";

        const double snr_db = 20.0;
        const int tx_frames = 200;
        const double fc = 2.45e9;
        const double rate = 50000.0;
        const int hop = 1;

        const std::vector<ModulationType> mods = {
            ModulationType::QPSK,
            ModulationType::FSK,
            ModulationType::OOK,
            ModulationType::BPSK,
            ModulationType::QAM,
            ModulationType::FM,
            ModulationType::MSK
        };
        const std::vector<int> stos = { 10000, 100000 };
        const std::vector<double> cfos = { 9000.0, 12000.0 };
        const std::vector<double> sfos = { 5.0 };

        for (ModulationType mod : mods) {
            for (int sto : stos) {
                for (double cfo : cfos) {
                    for (double sfo : sfos) {
                        ChannelConfig ch;
                        ch.enable_awgn = true;
                        ch.snr_dB = snr_db;
                        ch.seed = 123;

                        ch.enable_sto = true;
                        ch.sto_samp = sto;

                        ch.enable_cfo = true;
                        ch.cfo_hz = cfo;

                        ch.enable_sfo = true;
                        ch.sfo_ppm = sfo;

                        TestResult result = run_channel_test(
                            snr_db,
                            tx_frames,
                            fc,
                            mod,
                            rate,
                            hop,
                            ch
                        );

                        std::cout << modulation_to_string(mod)
                            << " | STO=" << ch.sto_samp << " samp"
                            << " | CFO=" << std::fixed << std::setprecision(3) << ch.cfo_hz << " Hz"
                            << " | SFO=" << ch.sfo_ppm << " ppm"
                            << " | Frames=" << tx_frames
                            << " | BER=" << std::scientific << std::setprecision(6) << result.total_ber
                            << std::fixed << " | bit_errors=" << result.total_bit_errors
                            << " | compared_bits=" << result.total_compared_bits
                            << " | decoded_frames=" << result.decoded_frames
                            << "\n";
                    }
                }
            }
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "ERROR: " << e.what() << std::endl;
    }

    return 0;
}
