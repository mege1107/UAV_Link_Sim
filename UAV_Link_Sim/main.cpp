#include <iostream>
#include "sim_runner.h"

int main()
{
    try
    {
        TestResult tr = run_file_transfer_test(
            RunMode::AWGN,                // 先测 LOOPBACK
            19.0,                            // LOOPBACK 这里无所谓
            2.45e9,                           // 中心频率，LOOPBACK 下基本无所谓
            ModulationType::QAM,             // 先用最稳的 BPSK
            "D:/linksim/test_files/kun.mp4", // 输入文件
            "D:/linksim/test_files/outkun.mp4"   // 输出文件
        );

        std::cout << "========== FILE TRANSFER RESULT ==========\n";
        std::cout << tr.log_text << std::endl;
        std::cout << "file_saved = " << (tr.file_saved ? "true" : "false") << std::endl;
        std::cout << "saved_file_path = " << tr.saved_file_path << std::endl;
    }
    catch (const std::exception& e)
    {
        std::cerr << "ERROR: " << e.what() << std::endl;
    }

    return 0;
}