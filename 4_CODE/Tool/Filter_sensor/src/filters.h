#pragma once
#include <algorithm>
#include <cmath>
#include <complex>
#include <vector>

constexpr float kPi = 3.14159265358979323846f;
constexpr float kRad2Deg = 180.0f / kPi;

// 1st-order IIR low-pass: y += alpha * (x - y)
struct LowPass1 {
    float alpha = 1.f, y = 0.f;
    bool init = false;
    void design(float fc, float fs) {
        const float dt = 1.f / fs;
        const float rc = 1.f / (2.f * kPi * std::max(fc, 1e-3f));
        alpha = dt / (rc + dt);
        init = false;
    }
    float step(float x) {
        if (!init) { y = x; init = true; }
        else y += alpha * (x - y);
        return y;
    }
};

// 2nd-order Butterworth low-pass (biquad, direct form I)
struct Biquad {
    float b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0;
    float x1 = 0, x2 = 0, y1 = 0, y2 = 0;
    bool init = false;
    void design(float fc, float fs) {
        fc = std::clamp(fc, 0.01f, fs * 0.49f);
        const float w0 = 2.f * kPi * fc / fs;
        const float c = std::cos(w0), s = std::sin(w0);
        const float alpha = s / (2.f * 0.70710678f);
        const float a0 = 1.f + alpha;
        b0 = (1.f - c) * 0.5f / a0;
        b1 = (1.f - c) / a0;
        b2 = b0;
        a1 = -2.f * c / a0;
        a2 = (1.f - alpha) / a0;
        init = false;
    }
    float step(float x) {
        if (!init) { x1 = x2 = y1 = y2 = x; init = true; }
        const float y = b0 * x + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
        x2 = x1; x1 = x;
        y2 = y1; y1 = y;
        return y;
    }
};

// Moving average (boxcar)
struct MovingAverage {
    std::vector<float> buf;
    size_t idx = 0, count = 0;
    double sum = 0;
    void design(int n) {
        buf.assign(static_cast<size_t>(std::max(1, n)), 0.f);
        idx = count = 0;
        sum = 0;
    }
    float step(float x) {
        sum += x - buf[idx];
        buf[idx] = x;
        idx = (idx + 1) % buf.size();
        if (count < buf.size()) ++count;
        return static_cast<float>(sum / static_cast<double>(count));
    }
};

// Complementary filter: angle = a*(angle + rate*dt) + (1-a)*accAngle,  a = tau/(tau+dt)
struct Complementary {
    float alpha = 0.98f, angle = 0.f;
    bool init = false;
    void design(float tau, float fs) {
        const float dt = 1.f / fs;
        alpha = tau / (tau + dt);
        init = false;
    }
    float step(float accAngle, float rate, float dt) {
        if (!init) { angle = accAngle; init = true; return angle; }
        angle = alpha * (angle + rate * dt) + (1.f - alpha) * accAngle;
        return angle;
    }
};

// 1-D Kalman filter for angle + gyro bias (state: [angle, bias])
struct Kalman {
    float qAngle = 0.001f, qBias = 0.003f, rMeasure = 0.03f;
    float angle = 0.f, bias = 0.f;
    float P00 = 0, P01 = 0, P10 = 0, P11 = 0;
    bool init = false;
    void design(float qa, float qb, float r) {
        qAngle = qa; qBias = qb; rMeasure = r;
        angle = bias = 0.f;
        P00 = P01 = P10 = P11 = 0.f;
        init = false;
    }
    float step(float newAngle, float newRate, float dt) {
        if (!init) { angle = newAngle; init = true; return angle; }
        const float rate = newRate - bias;
        angle += dt * rate;

        P00 += dt * (dt * P11 - P01 - P10 + qAngle);
        P01 -= dt * P11;
        P10 -= dt * P11;
        P11 += qBias * dt;

        const float S = P00 + rMeasure;
        const float K0 = P00 / S, K1 = P10 / S;
        const float y = newAngle - angle;
        angle += K0 * y;
        bias += K1 * y;

        const float p00 = P00, p01 = P01;
        P00 -= K0 * p00;
        P01 -= K0 * p01;
        P10 -= K1 * p00;
        P11 -= K1 * p01;
        return angle;
    }
};

// ---- FFT (radix-2, N must be a power of two) --------------------------------
inline void fftInPlace(std::vector<std::complex<float>>& a) {
    const size_t n = a.size();
    for (size_t i = 1, j = 0; i < n; ++i) {
        size_t bit = n >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) std::swap(a[i], a[j]);
    }
    for (size_t len = 2; len <= n; len <<= 1) {
        const float ang = -2.f * kPi / static_cast<float>(len);
        for (size_t i = 0; i < n; i += len) {
            for (size_t j = 0; j < len / 2; ++j) {
                const std::complex<float> w = std::polar(1.f, ang * static_cast<float>(j));
                const std::complex<float> u = a[i + j];
                const std::complex<float> v = a[i + j + len / 2] * w;
                a[i + j] = u + v;
                a[i + j + len / 2] = u - v;
            }
        }
    }
}

// Single-sided amplitude spectrum (DC removed, Hann window). x.size() must be 2^n.
inline void amplitudeSpectrum(const std::vector<float>& x, float fs,
                              std::vector<float>& freq, std::vector<float>& mag) {
    const size_t N = x.size();
    double mean = 0;
    for (float v : x) mean += v;
    mean /= static_cast<double>(N);

    std::vector<std::complex<float>> a(N);
    double sumW = 0;
    for (size_t i = 0; i < N; ++i) {
        const float w = 0.5f - 0.5f * std::cos(2.f * kPi * static_cast<float>(i) / static_cast<float>(N - 1));
        a[i] = {(x[i] - static_cast<float>(mean)) * w, 0.f};
        sumW += w;
    }
    fftInPlace(a);
    const size_t H = N / 2;
    freq.resize(H);
    mag.resize(H);
    for (size_t k = 0; k < H; ++k) {
        freq[k] = static_cast<float>(k) * fs / static_cast<float>(N);
        mag[k] = static_cast<float>(2.0 * std::abs(a[k]) / sumW);
    }
}
