#include <iostream>
#include <iomanip>

#include "sim_runner.h"

int main()
{
    TestResult tr = run_one_test(
        RunMode::AWGN,
        10.0,
        200,
        2.45e9,
        ModulationType::BPSK
    );

    std::cout << tr.log_text;

    std::cout
        << "decoded_frames = " << tr.decoded_frames
        << " BER = " << std::setprecision(6)
        << tr.total_ber
        << std::endl;

    return 0;
}