#include <iostream>
#include "sim_runner.h"

int main()
{
    try
    {
        TestResult tr = run_one_test(
            RunMode::AWGN,          // AWGN 信道
            0.0,                   // SNR（你可以改，比如 -10 ~ 10）
            100,                    // 重复帧数（越大越准，建议 >=100）
            2.45e9,                 // 中心频率（AWGN下无影响）
            ModulationType::MSK,     // 调制方式（你现在重点测这个）
            64000,
            1
        );

        std::cout << "========== BER TEST ==========\n";
        std::cout << tr.log_text << std::endl;

        std::cout << "Total bits = " << tr.total_compared_bits << std::endl;
        std::cout << "Bit errors = " << tr.total_bit_errors << std::endl;
        std::cout << "BER = " << tr.total_ber << std::endl;
    }
    catch (const std::exception& e)
    {
        std::cerr << "ERROR: " << e.what() << std::endl;
    }

    return 0;
}