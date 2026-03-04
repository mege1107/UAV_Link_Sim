#include <iostream>
#include "transmitter.h"
#include "receiver.h"
#include "channel.h"

using namespace std;

double computeBER(const VecInt& tx,
    const VecInt& rx)
{
    int N = min(tx.size(), rx.size());
    int err = 0;

    for (int i = 0; i < N; ++i)
        if (tx[i] != rx[i])
            err++;

    cout << "Total bits: " << N << endl;
    cout << "Bit errors: " << err << endl;

    return (double)err / N;
}

void printBits(const VecInt& v,
    const string& name,
    int count = 20)
{
    cout << name << " (first "
        << count << " bits): ";

    for (int i = 0; i < count && i < v.size(); ++i)
        cout << v[i];

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

    // 发射
    VecComplex tx_sig = tx.generateTransmitSignal();
    VecInt tx_bits = tx.getLastSourceBits();

    // 信道
    VecComplex rx_sig =
        Channel::awgn(tx_sig, -100);

    // 接收
    VecInt rx_bits = rx.receive(rx_sig);

    printBits(tx_bits, "TX bits");
    printBits(rx_bits, "RX bits");

    double ber = computeBER(tx_bits, rx_bits);

    cout << "BER = " << ber << endl;
    for (double snr = -5; snr <= 0; snr += 1)
    {
        VecComplex rx_sig = Channel::awgn(tx_sig, snr);
        VecInt rx_bits = rx.receive(rx_sig);
        double ber = computeBER(tx_bits, rx_bits);

        cout << "SNR = " << snr
            << " dB, BER = "
            << ber << endl;
    }
    return 0;
}
