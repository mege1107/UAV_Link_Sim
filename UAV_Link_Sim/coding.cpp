#include "coding.h"

#include <stdexcept>
#include <algorithm>
#include <cstddef>

namespace {

    // ============================================================
    // RS(31,15) over GF(2^5)
    // primitive polynomial: x^5 + x^2 + 1  -> 0b100101 -> 0x25
    // narrow-sense RS: roots = alpha^1 ... alpha^16
    // ============================================================

    static const int RS_M = 5;
    static const int RS_N = 31;
    static const int RS_K = 15;
    static const int RS_NROOTS = RS_N - RS_K;   // 16
    static const int RS_A0 = RS_N;              // log-domain zero marker
    static const int RS_PRIM_POLY = 0x25;       // x^5 + x^2 + 1

    struct RSCodec {
        int alpha_to[RS_N + 1];
        int index_of[RS_N + 1];
        int genpoly[RS_NROOTS + 1]; // conventional form, degree 16
    };

    inline int modnn(int x) {
        x %= RS_N;
        if (x < 0) x += RS_N;
        return x;
    }

    inline int gf_mul(const RSCodec& rs, int a, int b) {
        if (a == 0 || b == 0) return 0;
        return rs.alpha_to[modnn(rs.index_of[a] + rs.index_of[b])];
    }

    inline int gf_div(const RSCodec& rs, int a, int b) {
        if (a == 0) return 0;
        if (b == 0) throw std::runtime_error("RS gf_div divide by zero");
        return rs.alpha_to[modnn(rs.index_of[a] - rs.index_of[b])];
    }

    std::vector<int> polyMul(const RSCodec& rs, const std::vector<int>& a, const std::vector<int>& b) {
        std::vector<int> c(a.size() + b.size() - 1, 0);
        for (size_t i = 0; i < a.size(); ++i) {
            if (a[i] == 0) continue;
            for (size_t j = 0; j < b.size(); ++j) {
                if (b[j] == 0) continue;
                c[i + j] ^= gf_mul(rs, a[i], b[j]);
            }
        }
        return c;
    }

    RSCodec buildRSCodec() {
        RSCodec rs{};

        // 1) build GF tables
        int x = 1;
        for (int i = 0; i < RS_N; ++i) {
            rs.alpha_to[i] = x;
            rs.index_of[x] = i;

            x <<= 1;
            if (x & (1 << RS_M)) {
                x ^= RS_PRIM_POLY;
            }
            x &= RS_N; // keep 5 bits
        }
        rs.alpha_to[RS_N] = 1;
        rs.index_of[0] = RS_A0;

        // 2) generator polynomial g(x)=prod_{i=1}^{16}(x + alpha^i)
        std::vector<int> g(1, 1);
        for (int i = 1; i <= RS_NROOTS; ++i) {
            std::vector<int> factor;
            factor.push_back(1);
            factor.push_back(rs.alpha_to[i]);
            g = polyMul(rs, g, factor);
        }

        for (int i = 0; i <= RS_NROOTS; ++i) {
            rs.genpoly[i] = g[i];
        }
        return rs;
    }

    const RSCodec& getRS() {
        static RSCodec rs = buildRSCodec();
        return rs;
    }

    bool isSupportedRS(int n, int k) {
        return (n == RS_N && k == RS_K);
    }

    std::vector<int> bitsToSymbols5(const VecInt& bits) {
        if (bits.empty()) return {};
        if (bits.size() % RS_M != 0) {
            throw std::runtime_error("RS bitsToSymbols5: bit length must be multiple of 5");
        }

        std::vector<int> symbols(bits.size() / RS_M, 0);
        for (size_t i = 0; i < symbols.size(); ++i) {
            int v = 0;
            for (int b = 0; b < RS_M; ++b) {
                v = (v << 1) | (bits[i * RS_M + (size_t)b] & 1);
            }
            symbols[i] = v;
        }
        return symbols;
    }

    VecInt symbolsToBits5(const std::vector<int>& symbols) {
        VecInt bits;
        bits.reserve(symbols.size() * RS_M);
        for (size_t i = 0; i < symbols.size(); ++i) {
            int v = symbols[i] & RS_N; // 5 bits
            for (int b = RS_M - 1; b >= 0; --b) {
                bits.push_back((v >> b) & 1);
            }
        }
        return bits;
    }

    std::vector<int> computeParity(const RSCodec& rs, const std::vector<int>& msg) {
        // Polynomial long division on conventional-form symbols
        // codeword = msg || parity
        std::vector<int> work = msg;
        work.resize(msg.size() + RS_NROOTS, 0);

        for (size_t i = 0; i < msg.size(); ++i) {
            int coef = work[i];
            if (coef == 0) continue;

            for (int j = 1; j <= RS_NROOTS; ++j) {
                work[i + (size_t)j] ^= gf_mul(rs, rs.genpoly[j], coef);
            }
        }

        std::vector<int> parity(RS_NROOTS, 0);
        for (int i = 0; i < RS_NROOTS; ++i) {
            parity[i] = work[msg.size() + (size_t)i];
        }
        return parity;
    }

    std::vector<int> computeSyndromes(const RSCodec& rs, const std::vector<int>& codeword) {
        std::vector<int> s(RS_NROOTS, 0);

        for (int root = 1; root <= RS_NROOTS; ++root) {
            int accum = 0;
            for (size_t j = 0; j < codeword.size(); ++j) {
                accum = gf_mul(rs, accum, rs.alpha_to[root]) ^ codeword[j];
            }
            s[root - 1] = accum;
        }
        return s;
    }

    bool allZero(const std::vector<int>& v) {
        for (size_t i = 0; i < v.size(); ++i) {
            if (v[i] != 0) return false;
        }
        return true;
    }

    void trimPoly(std::vector<int>& p) {
        while (p.size() > 1 && p.back() == 0) {
            p.pop_back();
        }
    }

    std::vector<int> berlekampMassey(const RSCodec& rs, const std::vector<int>& synd, int& out_L) {
        std::vector<int> C(RS_NROOTS + 1, 0);
        std::vector<int> B(RS_NROOTS + 1, 0);

        C[0] = 1;
        B[0] = 1;

        int L = 0;
        int m = 1;
        int b = 1;

        for (int n = 0; n < RS_NROOTS; ++n) {
            int d = synd[n];
            for (int i = 1; i <= L; ++i) {
                if (C[i] != 0 && synd[n - i] != 0) {
                    d ^= gf_mul(rs, C[i], synd[n - i]);
                }
            }

            if (d == 0) {
                ++m;
                continue;
            }

            std::vector<int> T = C;
            int coef = gf_div(rs, d, b);

            for (int i = 0; i + m <= RS_NROOTS; ++i) {
                if (B[i] != 0) {
                    C[i + m] ^= gf_mul(rs, coef, B[i]);
                }
            }

            if (2 * L <= n) {
                L = n + 1 - L;
                B = T;
                b = d;
                m = 1;
            }
            else {
                ++m;
            }
        }

        trimPoly(C);
        out_L = L;
        return C;
    }

    int polyEval(const RSCodec& rs, const std::vector<int>& poly, int x) {
        int y = 0;
        for (int i = (int)poly.size() - 1; i >= 0; --i) {
            y = gf_mul(rs, y, x) ^ poly[(size_t)i];
        }
        return y;
    }

    std::vector<int> polyMulTrunc(const RSCodec& rs,
        const std::vector<int>& a,
        const std::vector<int>& b,
        int max_deg_exclusive) {
        std::vector<int> c((size_t)max_deg_exclusive, 0);
        for (size_t i = 0; i < a.size(); ++i) {
            if (a[i] == 0) continue;
            for (size_t j = 0; j < b.size(); ++j) {
                if (b[j] == 0) continue;
                size_t deg = i + j;
                if ((int)deg >= max_deg_exclusive) break;
                c[deg] ^= gf_mul(rs, a[i], b[j]);
            }
        }
        return c;
    }

    std::vector<int> findErrorPositions(const RSCodec& rs, const std::vector<int>& lambda) {
        // Chien search:
        // if lambda(alpha^{-j}) == 0, error is at position RS_N-1-j
        std::vector<int> err_pos;
        for (int j = 0; j < RS_N; ++j) {
            int x = rs.alpha_to[modnn(RS_N - j)]; // alpha^{-j}
            if (polyEval(rs, lambda, x) == 0) {
                err_pos.push_back(RS_N - 1 - j);
            }
        }
        return err_pos;
    }

    int evalFormalDerivativeAt(const RSCodec& rs, const std::vector<int>& lambda, int x) {
        // characteristic 2: derivative keeps only odd-order terms
        int den = 0;
        int x2 = gf_mul(rs, x, x);
        int power = 1; // x^(0*2)

        for (size_t idx = 1; idx < lambda.size(); idx += 2) {
            if (lambda[idx] != 0) {
                den ^= gf_mul(rs, lambda[idx], power);
            }
            power = gf_mul(rs, power, x2);
        }
        return den;
    }

    std::vector<int> decodeRS31_15(const std::vector<int>& rx_symbols, bool& success) {
        const RSCodec& rs = getRS();
        success = false;

        if ((int)rx_symbols.size() != RS_N) {
            return {};
        }

        std::vector<int> codeword = rx_symbols;
        std::vector<int> synd = computeSyndromes(rs, codeword);

        if (allZero(synd)) {
            success = true;
            return std::vector<int>(codeword.begin(), codeword.begin() + RS_K);
        }

        int L = 0;
        std::vector<int> lambda = berlekampMassey(rs, synd, L);
        std::vector<int> err_pos = findErrorPositions(rs, lambda);

        if ((int)err_pos.size() != L || L > RS_NROOTS / 2) {
            return std::vector<int>(codeword.begin(), codeword.begin() + RS_K);
        }

        std::vector<int> omega = polyMulTrunc(rs, synd, lambda, RS_NROOTS);

        for (size_t t = 0; t < err_pos.size(); ++t) {
            int pos = err_pos[t];
            int j = RS_N - 1 - pos;
            int x = rs.alpha_to[modnn(RS_N - j)]; // alpha^{-j}

            int num = polyEval(rs, omega, x);
            int den = evalFormalDerivativeAt(rs, lambda, x);

            if (den == 0) {
                return std::vector<int>(codeword.begin(), codeword.begin() + RS_K);
            }

            int err_mag = gf_div(rs, num, den);
            codeword[(size_t)pos] ^= err_mag;
        }

        std::vector<int> check = computeSyndromes(rs, codeword);
        if (!allZero(check)) {
            return std::vector<int>(codeword.begin(), codeword.begin() + RS_K);
        }

        success = true;
        return std::vector<int>(codeword.begin(), codeword.begin() + RS_K);
    }

} // namespace

// ========================================
// RS编码
// 当前项目使用 RS(31,15) over GF(2^5)
// 输入 bit 流长度应为 75 bit
// 输出 bit 流长度为 155 bit
// ========================================
VecInt HXL_RSCode(const VecInt& data, int n, int k)
{
    if (!isSupportedRS(n, k)) {
        return data;
    }

    try {
        const int rs_msg_bits = RS_K * RS_M;   // 15*5 = 75
        const int rs_code_bits = RS_N * RS_M;  // 31*5 = 155

        if (data.empty()) return {};
        if ((int)data.size() % rs_msg_bits != 0) {
            // 当前工程里整帧应当正好是75的整数倍
            return data;
        }

        VecInt out;
        out.reserve((data.size() / rs_msg_bits) * rs_code_bits);

        const RSCodec& rs = getRS();

        for (size_t off = 0; off < data.size(); off += (size_t)rs_msg_bits) {
            VecInt blk_bits(data.begin() + off, data.begin() + off + rs_msg_bits);

            std::vector<int> msg = bitsToSymbols5(blk_bits);   // 15 symbols
            std::vector<int> parity = computeParity(rs, msg);  // 16 symbols

            std::vector<int> codeword;
            codeword.reserve(RS_N);
            codeword.insert(codeword.end(), msg.begin(), msg.end());
            codeword.insert(codeword.end(), parity.begin(), parity.end());

            VecInt code_bits = symbolsToBits5(codeword);       // 155 bits
            out.insert(out.end(), code_bits.begin(), code_bits.end());
        }

        return out;
    }
    catch (...) {
        return data;
    }
}

// ========================================
// RS解码
// 输入 bit 流长度应为 155 bit
// 输出 bit 流长度为 75 bit
// 若纠错失败，返回“前15个符号对应的信息位”
// ========================================
VecInt HXL_RSDecode(const VecInt& data, int n, int k)
{
    if (!isSupportedRS(n, k)) {
        return data;
    }

    try {
        const int rs_msg_bits = RS_K * RS_M;   // 75
        const int rs_code_bits = RS_N * RS_M;  // 155

        if (data.empty()) return {};
        if ((int)data.size() % rs_code_bits != 0) {
            return data;
        }

        VecInt out;
        out.reserve((data.size() / rs_code_bits) * rs_msg_bits);

        for (size_t off = 0; off < data.size(); off += (size_t)rs_code_bits) {
            VecInt blk_bits(data.begin() + off, data.begin() + off + rs_code_bits);

            std::vector<int> rx_symbols = bitsToSymbols5(blk_bits); // 31 symbols
            bool ok = false;
            std::vector<int> dec = decodeRS31_15(rx_symbols, ok);   // 15 symbols

            if ((int)dec.size() != RS_K) {
                return out; // 或者返回data，看你想怎么处理失败
            }

            VecInt msg_bits = symbolsToBits5(dec);                  // 75 bits
            out.insert(out.end(), msg_bits.begin(), msg_bits.end());
        }

        return out;
    }
    catch (...) {
        return data;
    }
}

// ========================================
// 差分编码
// ========================================
VecInt d_encode(const VecInt& data)
{
    if (data.empty()) return {};

    VecInt diff(data.size());
    diff[0] = data[0] & 1;

    for (size_t i = 1; i < data.size(); ++i)
        diff[i] = (data[i] & 1) ^ (diff[i - 1] & 1);

    return diff;
}

// ========================================
// 差分解码
// ========================================
VecInt d_decode(const VecInt& diff)
{
    if (diff.empty()) return {};

    VecInt data(diff.size());
    data[0] = diff[0] & 1;

    for (size_t i = 1; i < diff.size(); ++i)
        data[i] = (diff[i] & 1) ^ (diff[i - 1] & 1);

    return data;
}