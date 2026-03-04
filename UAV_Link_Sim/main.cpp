#include <iostream>
#include <algorithm> // std::min
#include "transmitter.h"
#include "receiver.h"
#include "channel.h"

using namespace std;

double computeBER(const VecInt& tx, const VecInt& rx)
{
    int N = static_cast<int>(std::min(tx.size(), rx.size()));
    if (N <= 0) {
        cout << "Total bits: 0" << endl;
        cout << "Bit errors: 0" << endl;
        return 0.0;
    }

    int err = 0;
    for (int i = 0; i < N; ++i) {
        if (tx[i] != rx[i]) err++;
    }

    cout << "Total bits: " << N << endl;
    cout << "Bit errors: " << err << endl;
    return static_cast<double>(err) / static_cast<double>(N);
}

void printBits(const VecInt& v, const string& name, int count = 20)
{
    cout << name << " (first " << count << " bits): ";
    for (int i = 0; i < count && i < static_cast<int>(v.size()); ++i) {
        cout << v[i];
    }
    cout << endl;
}

int main()
{
    TransmitterConfig cfg;
    cfg.function = FunctionType::Telemetry;
    cfg.modulation = ModulationType::BPSK;
    cfg.n = 1;
    cfg.frame_bit = 100000;
    cfg.samp = 4;

    Transmitter tx(cfg);
    Receiver rx(cfg);

    // 如需每次运行都得到相同噪声（可复现实验），保留这一句；
    // 如果你希望每次运行噪声不同，可以注释掉，或者用 random_device 生成 seed。
    Channel::setSeed(123);

    // 发射
    VecComplex tx_sig = tx.generateTransmitSignal();
    VecInt tx_bits = tx.getLastSourceBits();

    // 信道（你原来这里写的是 -100 dB :contentReference[oaicite:2]{index=2}，这会非常“几乎全是噪声”）
    VecComplex rx_sig = Channel::awgn(tx_sig, 100);

    // 接收
    VecInt rx_bits = rx.receive(rx_sig);

    printBits(tx_bits, "TX bits");
    printBits(rx_bits, "RX bits");

    double ber = computeBER(tx_bits, rx_bits);
    cout << "BER = " << ber << endl;

    for (double snr = -5; snr <= 0; snr += 1)
    {
        VecComplex rx_sig2 = Channel::awgn(tx_sig, snr);
        VecInt rx_bits2 = rx.receive(rx_sig2);
        double ber2 = computeBER(tx_bits, rx_bits2);

        cout << "SNR = " << snr << " dB, BER = " << ber2 << endl;
    }

    return 0;
}