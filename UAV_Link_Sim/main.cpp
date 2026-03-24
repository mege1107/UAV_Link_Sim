#include <iostream>
#include <iomanip>
#include <algorithm>
#include <fstream>
#include <iterator>

#include "ofdm_link.h"
#include "channel.h"

int main()
{
    try
    {
        OFDMConfig cfg;
        cfg.N_fft = 256;
        cfg.N_cp = 32;
        cfg.N_sc = 128;
        cfg.Nd = 10;
        cfg.N_frm = 1;
        cfg.N_zeros = 256;
        cfg.P_f_inter = 6;
        cfg.M = 4;
        cfg.L = 7;
        cfg.tblen = 32;
        cfg.delta_f = 15e3;

        const std::string input_path = "D:\\linksim\\test_files\\test2.jpg";
        const std::string output_path = "D:\\linksim\\test_files\\out9.jpg";

        std::ifstream ifs(input_path, std::ios::binary);
        if (!ifs) {
            throw std::runtime_error("Cannot open input file: " + input_path);
        }

        std::vector<unsigned char> file_bytes{
            std::istreambuf_iterator<char>(ifs),
            std::istreambuf_iterator<char>()};
        if (file_bytes.empty()) {
            throw std::runtime_error("Input file is empty: " + input_path);
        }
        OFDMImageTransmitter tx(cfg);
        OFDMImageReceiver rx(cfg);
        std::vector<VecComplex> tx_frames = tx.buildFileFrames("test2.jpg", file_bytes);
        VecComplex tx_sig = tx.buildFileSignal("test2.jpg", file_bytes);

        ChannelConfig ch;
        ch.enable_awgn = true;
        ch.snr_dB = 20.0;
        ch.seed = 123;
        ch.enable_sto = false;
        ch.enable_cfo = false;
        ch.enable_sfo = false;

        VecComplex rx_sig = Channel::process(tx_sig, ch, cfg.sampleRate());

        std::string rx_name;
        std::vector<uint8_t> rx_file_bytes;
        const bool ok = rx.receiveFileSignal(rx_sig, rx_name, rx_file_bytes);

        std::cout << "============================\n";
        std::cout << "  OFDM FILE-LIKE IMAGE TEST\n";
        std::cout << "============================\n";
        std::cout << std::fixed << std::setprecision(3)
            << "SNR=" << ch.snr_dB << " dB"
            << " | input_bytes=" << file_bytes.size()
            << " | tx_frames=" << tx_frames.size()
            << " | rx_ok=" << (ok ? 1 : 0)
            << " | rx_name=" << rx_name
            << " | rx_bytes=" << rx_file_bytes.size()
            << "\n";

        if (ok) {
            size_t diff_cnt = 0;
            const size_t n = std::min(file_bytes.size(), rx_file_bytes.size());
            for (size_t i = 0; i < n; ++i) {
                if (file_bytes[i] != rx_file_bytes[i]) diff_cnt++;
            }

            std::cout << "byte_errors=" << diff_cnt
                << " | compared_bytes=" << n
                << "\n";

            std::ofstream ofs(output_path, std::ios::binary);
            if (!ofs) {
                throw std::runtime_error("Cannot open output file: " + output_path);
            }
            ofs.write(
                reinterpret_cast<const char*>(rx_file_bytes.data()),
                static_cast<std::streamsize>(rx_file_bytes.size()));
            ofs.close();

            std::cout << "saved_to=" << output_path << "\n";
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "ERROR: " << e.what() << std::endl;
    }

    return 0;
}
