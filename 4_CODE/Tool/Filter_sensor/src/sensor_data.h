#pragma once
#include <array>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include "filters.h"

inline float accLsbPerG(int range) {
    static const float v[4] = {16384.f, 8192.f, 4096.f, 2048.f};
    return v[range & 3];
}
inline float gyroLsbPerDps(int range) {
    static const float v[4] = {131.f, 65.5f, 32.8f, 16.4f};
    return v[range & 3];
}

struct Config {
    float fs = 200.f;       // sample rate of the frames (Hz) -> defines dt / time axis
    int accRange = 0;       // 0..3 = +/-2,4,8,16 g
    int gyroRange = 0;      // 0..3 = +/-250,500,1000,2000 dps
    int angleAxis = 0;      // 0 = roll (atan2(ay,az), gyro X), 1 = pitch (atan2(-ax,|ay,az|), gyro Y)
    bool invertGyro = false;
    float gyroBias[3] = {0, 0, 0};  // dps, subtracted from gyro

    // single-channel filter design
    int chan = 3;           // 0..5 = ax ay az gx gy gz
    int filterType = 0;     // 0 = IIR 1st order, 1 = Butterworth 2nd order, 2 = moving average
    float cutoffHz = 10.f;
    int maWindow = 8;

    // angle estimators
    float compTau = 0.5f;   // seconds
    float kalQAngle = 0.001f, kalQBias = 0.003f, kalR = 0.03f;
};

class SensorData {
public:
    static constexpr size_t kMaxSamples = 300000;

    Config cfg;
    std::vector<float> t;                      // seconds
    std::array<std::vector<float>, 6> raw;     // counts
    std::array<std::vector<float>, 6> phys;    // g, g, g, dps, dps, dps
    std::vector<float> chanFilt;               // filtered selected channel (physical units)
    std::vector<float> accAng, gyroAng, comp, kal;  // degrees
    uint64_t firstIndex = 0;

    SensorData() { designFilters(); }

    size_t size() const { return t.size(); }

    void clear() {
        for (auto* v : all()) v->clear();
        firstIndex = 0;
        designFilters();
    }

    void append(const int16_t v[6]) {
        float r[6];
        for (int k = 0; k < 6; ++k) {
            r[k] = static_cast<float>(v[k]);
            raw[k].push_back(r[k]);
        }
        t.push_back(static_cast<float>(static_cast<double>(firstIndex + t.size()) / cfg.fs));
        derive(r);
        if (t.size() > kMaxSamples + kMaxSamples / 10) trim(kMaxSamples / 10);
    }

    // Re-run the whole pipeline over the stored raw samples (after changing any parameter).
    void recomputeAll() {
        designFilters();
        for (auto& v : phys) v.clear();
        chanFilt.clear(); accAng.clear(); gyroAng.clear(); comp.clear(); kal.clear();
        const size_t n = t.size();
        for (size_t i = 0; i < n; ++i) {
            t[i] = static_cast<float>(static_cast<double>(firstIndex + i) / cfg.fs);
            float r[6];
            for (int k = 0; k < 6; ++k) r[k] = raw[k][i];
            derive(r);
        }
    }

    // Average gyro over the last `seconds` (sensor must be still) and store it as bias.
    bool calibrateGyro(float seconds) {
        const size_t n = std::min(size(), static_cast<size_t>(std::max(1.f, seconds * cfg.fs)));
        if (n < 10) return false;
        for (int k = 0; k < 3; ++k) {
            double s = 0;
            for (size_t i = size() - n; i < size(); ++i) s += raw[3 + k][i];
            cfg.gyroBias[k] = static_cast<float>(s / static_cast<double>(n)) / gyroLsbPerDps(cfg.gyroRange);
        }
        recomputeAll();
        return true;
    }

    bool saveCsv(const std::string& path) const {
        FILE* f = std::fopen(path.c_str(), "w");
        if (!f) return false;
        std::fprintf(f, "t,ax_raw,ay_raw,az_raw,gx_raw,gy_raw,gz_raw,"
                        "ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,chan_filt,acc_angle,gyro_angle,comp,kalman\n");
        for (size_t i = 0; i < size(); ++i) {
            std::fprintf(f, "%.6f", t[i]);
            for (int k = 0; k < 6; ++k) std::fprintf(f, ",%d", static_cast<int>(raw[k][i]));
            for (int k = 0; k < 6; ++k) std::fprintf(f, ",%.6f", phys[k][i]);
            std::fprintf(f, ",%.6f,%.6f,%.6f,%.6f,%.6f\n", chanFilt[i], accAng[i], gyroAng[i], comp[i], kal[i]);
        }
        std::fclose(f);
        return true;
    }

private:
    LowPass1 lp_;
    Biquad bq_;
    MovingAverage ma_;
    Complementary compF_;
    Kalman kalF_;
    float gyroInt_ = 0.f;
    bool gyroInit_ = false;

    std::vector<std::vector<float>*> all() {
        std::vector<std::vector<float>*> v{&t, &chanFilt, &accAng, &gyroAng, &comp, &kal};
        for (auto& a : raw) v.push_back(&a);
        for (auto& a : phys) v.push_back(&a);
        return v;
    }

    void trim(size_t cut) {
        for (auto* v : all()) v->erase(v->begin(), v->begin() + static_cast<std::ptrdiff_t>(cut));
        firstIndex += cut;
    }

    void designFilters() {
        lp_.design(cfg.cutoffHz, cfg.fs);
        bq_.design(cfg.cutoffHz, cfg.fs);
        ma_.design(cfg.maWindow);
        compF_.design(cfg.compTau, cfg.fs);
        kalF_.design(cfg.kalQAngle, cfg.kalQBias, cfg.kalR);
        gyroInit_ = false;
        gyroInt_ = 0.f;
    }

    void derive(const float r[6]) {
        const float dt = 1.f / cfg.fs;
        const float sgn = cfg.invertGyro ? -1.f : 1.f;
        float p[6];
        for (int k = 0; k < 3; ++k) p[k] = r[k] / accLsbPerG(cfg.accRange);
        for (int k = 0; k < 3; ++k)
            p[3 + k] = sgn * (r[3 + k] / gyroLsbPerDps(cfg.gyroRange) - cfg.gyroBias[k]);
        for (int k = 0; k < 6; ++k) phys[k].push_back(p[k]);

        float accA, rate;
        if (cfg.angleAxis == 0) {
            accA = std::atan2(p[1], p[2]) * kRad2Deg;
            rate = p[3];
        } else {
            accA = std::atan2(-p[0], std::sqrt(p[1] * p[1] + p[2] * p[2])) * kRad2Deg;
            rate = p[4];
        }
        accAng.push_back(accA);

        if (!gyroInit_) { gyroInt_ = accA; gyroInit_ = true; }
        else gyroInt_ += rate * dt;
        gyroAng.push_back(gyroInt_);

        comp.push_back(compF_.step(accA, rate, dt));
        kal.push_back(kalF_.step(accA, rate, dt));

        const float x = p[cfg.chan];
        const float y = cfg.filterType == 0 ? lp_.step(x) : cfg.filterType == 1 ? bq_.step(x) : ma_.step(x);
        chanFilt.push_back(y);
    }
};
