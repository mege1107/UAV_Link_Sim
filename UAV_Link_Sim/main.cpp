#include <iostream>
#include <stdexcept>

#include "role_runner.h"

// ===== 固定参数（两台电脑必须一致）=====
static const std::string DEVICE_ARGS = "type=b200";
static const double CENTER_FREQ = 433e6;
static const double INFO_RATE = 50000.0;
static const int HOP_PATTERN = 1;
static const ModulationType MOD = ModulationType::QAM;

// TX 参数
static const int TX_REPEAT = 5000;   // 多发几轮，防止对不上

// RX 参数
static const int RX_EXPECT_FRAMES = 300;

// 文件模式（可先不用）
static const bool USE_FILE_MODE = false;
static const std::string INPUT_FILE = "test.jpg";
static const std::string OUTPUT_FILE = "recv.jpg";

// ======================================

int main()
{
    try
    {
        std::cout << "============================\n";
        std::cout << "  Dual-PC USRP Test\n";
        std::cout << "============================\n";
        std::cout << "1. TX (Transmit)\n";
        std::cout << "2. RX (Receive)\n";
        std::cout << "Select: ";

        int choice = 0;
        std::cin >> choice;

        if (choice == 1)
        {
            std::cout << "\n[TX] Starting transmitter...\n";

            SourceMode src = USE_FILE_MODE ? SourceMode::FileBits : SourceMode::RandomBits;

            TxOnlyResult txr = run_tx_role_once(
                DEVICE_ARGS,
                CENTER_FREQ,
                MOD,
                INFO_RATE,
                HOP_PATTERN,
                src,
                INPUT_FILE,
                TX_REPEAT
            );

            std::cout << "\n========== TX DONE ==========\n";
            std::cout << txr.log_text << std::endl;
            std::cout << "TX frames = " << txr.tx_frame_count << std::endl;
            std::cout << "TX samples = " << txr.tx_sample_count << std::endl;
            std::cout << "fs = " << txr.fs << std::endl;
        }
        else if (choice == 2)
        {
            std::cout << "\n[RX] Starting receiver...\n";

            if (!USE_FILE_MODE)
            {
                RxOnlyResult rxr = run_rx_role_once(
                    DEVICE_ARGS,
                    CENTER_FREQ,
                    MOD,
                    INFO_RATE,
                    HOP_PATTERN,
                    RX_EXPECT_FRAMES
                );

                std::cout << "\n========== RX DONE ==========\n";
                std::cout << rxr.log_text << std::endl;
                std::cout << "RX samples = " << rxr.rx_sample_count << std::endl;
                std::cout << "Decoded bits = " << rxr.decoded_bits_count << std::endl;
                std::cout << "Decoded frames = " << rxr.decoded_frames << std::endl;
                std::cout << "Bit errors = " << rxr.total_bit_errors << std::endl;
                std::cout << "Compared bits = " << rxr.total_compared_bits << std::endl;
                std::cout << "BER = " << rxr.total_ber << std::endl;
                std::cout << "fs = " << rxr.fs << std::endl;
            }
            else
            {
                RxFileResult rfr = run_rx_file_role_once(
                    DEVICE_ARGS,
                    CENTER_FREQ,
                    MOD,
                    INFO_RATE,
                    HOP_PATTERN,
                    OUTPUT_FILE,
                    1000
                );

                std::cout << "\n========== RX FILE DONE ==========\n";
                std::cout << rfr.log_text << std::endl;
                std::cout << "File saved = " << (rfr.file_saved ? "YES" : "NO") << std::endl;
                std::cout << "Recovered filename = " << rfr.recovered_filename << std::endl;
                std::cout << "Saved path = " << rfr.saved_file_path << std::endl;
            }
        }
        else
        {
            throw std::runtime_error("Invalid choice.");
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "\nERROR: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
