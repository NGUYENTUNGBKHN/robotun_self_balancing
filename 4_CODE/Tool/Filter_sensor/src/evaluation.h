#pragma once
// Filter evaluation: measures DELAY and NOISE REDUCTION of angle estimators
// (Complementary, Kalman, LPF / Butterworth / moving-average applied to the accel angle).
//
// Method (works on live/recorded data, no external ground truth needed):
//   reference = accel angle filtered ZERO-PHASE (forward+backward Butterworth)
//               -> no delay, most of the noise removed.
//   delay     = lag that maximises cross-correlation(estimator, reference)   [ms]
//   noise     = std of ( estimator shifted back by the delay  -  reference )  [deg]
//   reduction = 20*log10( noise / noise_of_raw_accel_angle )                  [dB]
//   rmse      = RMS( estimator - reference )  (includes delay + attenuation) [deg]
//   gain      = std(estimator) / std(reference)  (<1: attenuating real motion)
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>
#include <vector>

#include "filters.h"
#include "sensor_data.h"

struct EvalParams {
    float windowSec = 10.f;      // length of the evaluated window (most recent data)
    float refCutoffHz = 2.f;     // zero-phase reference low-pass (>= ~3x highest motion freq)
    float maxLagMs = 200.f;      // search range for delay (keep < half the motion period)
    float minMotionDeg = 1.f;    // below this std the window is "static": delay not measurable
    bool sweep = true;           // also compute noise-vs-delay curves
};

struct EvalRow {
    std::string name;
    float delayMs = std::numeric_limits<float>::quiet_NaN();
    float noiseDeg = std::numeric_limits<float>::quiet_NaN();
    float reductionDb = std::numeric_limits<float>::quiet_NaN();
    float rmseDeg = std::numeric_limits<float>::quiet_NaN();
    float gain = std::numeric_limits<float>::quiet_NaN();
};

struct EvalCurve {
    std::string name;
    std::vector<float> param, delayMs, noiseDeg;
};

struct EvalResult {
    bool valid = false;
    std::string msg;
    bool moving = false;
    float motionStdDeg = 0.f;
    float usedWindowSec = 0.f;
    std::vector<EvalRow> rows;
    std::vector<EvalCurve> curves;
    std::vector<float> t;                      // time axis of the evaluated window
    std::vector<std::string> seriesNames;      // [0] = reference
    std::vector<std::vector<float>> series;
};

class FilterEval {
public:
    EvalParams p;
    EvalResult res;

    void run(const SensorData& S) {
        res = EvalResult{};
        fs_ = S.cfg.fs;
        const size_t n = S.size();
        const float refFc = std::clamp(p.refCutoffHz, 0.05f, fs_ * 0.2f);
        // margin: filter warm-up before the window + guard for the backward pass after it
        const size_t margin = static_cast<size_t>(std::ceil(std::clamp(3.f / refFc, 1.f, 5.f) * fs_));
        const size_t minW = static_cast<size_t>(2.f * fs_);
        if (n < 2 * margin + minW) {
            char b[160];
            std::snprintf(b, sizeof b, "Not enough data: need >= %.1f s in the buffer, have %.1f s.",
                static_cast<double>(2 * margin + minW) / fs_, static_cast<double>(n) / fs_);
            res.msg = b;
            return;
        }
        const size_t W = std::max(minW, std::min(static_cast<size_t>(p.windowSec * fs_), n - 2 * margin));
        L_ = W + 2 * margin;
        a_ = margin;
        b_ = margin + W;
        maxLag_ = std::max(1, std::min(static_cast<int>(p.maxLagMs * fs_ / 1000.f), static_cast<int>(margin) - 2));
        const size_t s0 = n - L_;
        const float dt = 1.f / fs_;

        const std::vector<float>& rateSrc = S.phys[S.cfg.angleAxis == 0 ? 3 : 4];
        acc_.assign(S.accAng.begin() + static_cast<std::ptrdiff_t>(s0), S.accAng.end());
        rate_.assign(rateSrc.begin() + static_cast<std::ptrdiff_t>(s0), rateSrc.end());
        ref_ = zeroPhase(acc_, refFc, fs_);

        // motion level of the reference inside the window
        {
            double m = 0, m2 = 0;
            for (size_t i = a_; i < b_; ++i) { m += ref_[i]; m2 += static_cast<double>(ref_[i]) * ref_[i]; }
            const double N = static_cast<double>(b_ - a_);
            refStd_ = static_cast<float>(std::sqrt(std::max(0.0, m2 / N - (m / N) * (m / N))));
        }
        moving_ = refStd_ >= p.minMotionDeg;

        // ---- estimators under test ------------------------------------------------
        struct Method { std::string name; std::vector<float> y; };
        std::vector<Method> ms;
        char nm[96];
        ms.push_back({ "Acc angle (raw)", acc_ });
        std::snprintf(nm, sizeof nm, "Complementary (tau %.3g s)", static_cast<double>(S.cfg.compTau));
        ms.push_back({ nm, slice(S.comp, s0) });
        std::snprintf(nm, sizeof nm, "Kalman (R %.3g)", static_cast<double>(S.cfg.kalR));
        ms.push_back({ nm, slice(S.kal, s0) });

        std::snprintf(nm, sizeof nm, "Acc + IIR 1st (%.3g Hz)", static_cast<double>(S.cfg.cutoffHz));
        {
            LowPass1 f; f.design(S.cfg.cutoffHz, fs_);
            std::vector<float> y(L_);
            for (size_t i = 0; i < L_; ++i) y[i] = f.step(acc_[i]);
            ms.push_back({ nm, std::move(y) });
        }
        std::snprintf(nm, sizeof nm, "Acc + Butterworth 2nd (%.3g Hz)", static_cast<double>(S.cfg.cutoffHz));
        {
            Biquad f; f.design(S.cfg.cutoffHz, fs_);
            std::vector<float> y(L_);
            for (size_t i = 0; i < L_; ++i) y[i] = f.step(acc_[i]);
            ms.push_back({ nm, std::move(y) });
        }
        std::snprintf(nm, sizeof nm, "Acc + Moving avg (N=%d)", S.cfg.maWindow);
        {
            MovingAverage f; f.design(S.cfg.maWindow);
            std::vector<float> y(L_);
            for (size_t i = 0; i < L_; ++i) y[i] = f.step(acc_[i]);
            ms.push_back({ nm, std::move(y) });
        }

        // ---- metrics -------------------------------------------------------------------
        for (const auto& m : ms) {
            const Metrics q = metrics(m.y);
            EvalRow r;
            r.name = m.name;
            r.delayMs = q.delayMs;
            r.noiseDeg = q.noise;
            r.rmseDeg = q.rmse;
            r.gain = q.gain;
            res.rows.push_back(std::move(r));
        }
        const float accNoise = res.rows[0].noiseDeg;
        for (auto& r : res.rows)
            r.reductionDb = 20.f * std::log10(std::max(r.noiseDeg, 1e-6f) / std::max(accNoise, 1e-6f));

        // ---- noise-vs-delay sweeps ----------------------------------------------------
        if (p.sweep && moving_) {
            const float fMax = std::min(fs_ * 0.45f, 60.f);
            const auto fcs = geom(0.5f, std::max(1.f, fMax), 16);
            std::vector<float> y(L_);

            EvalCurve c1; c1.name = "IIR 1st on acc (sweep fc)";
            for (float fc : fcs) {
                LowPass1 f; f.design(fc, fs_);
                for (size_t i = 0; i < L_; ++i) y[i] = f.step(acc_[i]);
                addPoint(c1, fc, y);
            }
            EvalCurve c2; c2.name = "Butterworth on acc (sweep fc)";
            for (float fc : fcs) {
                Biquad f; f.design(fc, fs_);
                for (size_t i = 0; i < L_; ++i) y[i] = f.step(acc_[i]);
                addPoint(c2, fc, y);
            }
            EvalCurve c3; c3.name = "Complementary (sweep tau)";
            for (float tau : geom(0.02f, 5.f, 16)) {
                Complementary f; f.design(tau, fs_);
                for (size_t i = 0; i < L_; ++i) y[i] = f.step(acc_[i], rate_[i], dt);
                addPoint(c3, tau, y);
            }
            EvalCurve c4; c4.name = "Kalman (sweep R)";
            for (float R : geom(1e-3f, 10.f, 16)) {
                Kalman f; f.design(S.cfg.kalQAngle, S.cfg.kalQBias, R);
                for (size_t i = 0; i < L_; ++i) y[i] = f.step(acc_[i], rate_[i], dt);
                addPoint(c4, R, y);
            }
            res.curves = { std::move(c1), std::move(c2), std::move(c3), std::move(c4) };
        }

        // ---- overlay data -----------------------------------------------------------
        res.t.assign(S.t.begin() + static_cast<std::ptrdiff_t>(s0 + a_), S.t.begin() + static_cast<std::ptrdiff_t>(s0 + b_));
        res.seriesNames.push_back("reference (zero-phase acc)");
        res.series.emplace_back(ref_.begin() + static_cast<std::ptrdiff_t>(a_), ref_.begin() + static_cast<std::ptrdiff_t>(b_));
        for (const auto& m : ms) {
            res.seriesNames.push_back(m.name);
            res.series.emplace_back(m.y.begin() + static_cast<std::ptrdiff_t>(a_), m.y.begin() + static_cast<std::ptrdiff_t>(b_));
        }
        res.moving = moving_;
        res.motionStdDeg = refStd_;
        res.usedWindowSec = static_cast<float>(b_ - a_) / fs_;
        res.valid = true;
    }

private:
    struct Metrics { float delayMs, noise, rmse, gain; };

    float fs_ = 200.f, refStd_ = 0.f;
    bool moving_ = false;
    size_t L_ = 0, a_ = 0, b_ = 0;
    int maxLag_ = 1;
    std::vector<float> acc_, rate_, ref_;

    static constexpr float nan() { return std::numeric_limits<float>::quiet_NaN(); }

    static std::vector<float> slice(const std::vector<float>& v, size_t s0) {
        return std::vector<float>(v.begin() + static_cast<std::ptrdiff_t>(s0), v.end());
    }

    static std::vector<float> geom(float lo, float hi, int n) {
        std::vector<float> v(static_cast<size_t>(n));
        for (int i = 0; i < n; ++i)
            v[static_cast<size_t>(i)] = lo * std::pow(hi / lo, static_cast<float>(i) / static_cast<float>(n - 1));
        return v;
    }

    // Forward + backward 2nd-order Butterworth => zero phase, (-3 dB at ~0.64*fc).
    static std::vector<float> zeroPhase(const std::vector<float>& x, float fc, float fs) {
        Biquad f; f.design(fc, fs);
        std::vector<float> y(x.size());
        for (size_t i = 0; i < x.size(); ++i) y[i] = f.step(x[i]);
        Biquad g; g.design(fc, fs);
        for (size_t i = y.size(); i-- > 0;) y[i] = g.step(y[i]);
        return y;
    }

    // linear interpolation with clamping
    static float sampleAt(const std::vector<float>& y, double x) {
        const double hi = static_cast<double>(y.size() - 1);
        x = std::clamp(x, 0.0, hi - 1e-9);
        const size_t i0 = static_cast<size_t>(x);
        const float f = static_cast<float>(x - static_cast<double>(i0));
        return y[i0] * (1.f - f) + y[i0 + 1] * f;
    }

    // Lag (in samples, sub-sample resolution) such that y[i + lag] ~ ref[i]; >0 => y is delayed.
    float xcorrLag(const std::vector<float>& y) const {
        const int st = std::max(1, static_cast<int>(fs_ / 250.f));
        const int K = std::max(1, maxLag_ / st);
        double my = 0, mr = 0;
        size_t cnt = 0;
        for (size_t i = a_; i < b_; i += static_cast<size_t>(st)) { my += y[i]; mr += ref_[i]; ++cnt; }
        my /= static_cast<double>(cnt);
        mr /= static_cast<double>(cnt);

        std::vector<double> r(static_cast<size_t>(2 * K + 1));
        for (int k = -K; k <= K; ++k) {
            double num = 0, syy = 0, srr = 0;
            for (size_t i = a_; i < b_; i += static_cast<size_t>(st)) {
                const double yy = y[static_cast<size_t>(static_cast<long long>(i) + static_cast<long long>(k) * st)] - my;
                const double rr = ref_[i] - mr;
                num += yy * rr; syy += yy * yy; srr += rr * rr;
            }
            r[static_cast<size_t>(k + K)] = num / std::sqrt(syy * srr + 1e-30);
        }
        int kb = 0;
        for (int k = 1; k < 2 * K + 1; ++k)
            if (r[static_cast<size_t>(k)] > r[static_cast<size_t>(kb)]) kb = k;
        double delta = 0;
        if (kb > 0 && kb < 2 * K) {
            const double rm = r[static_cast<size_t>(kb - 1)], r0 = r[static_cast<size_t>(kb)], rp = r[static_cast<size_t>(kb + 1)];
            const double den = rm - 2 * r0 + rp;
            if (std::fabs(den) > 1e-12) delta = std::clamp(0.5 * (rm - rp) / den, -1.0, 1.0);
        }
        return static_cast<float>((kb - K + delta) * st);
    }

    Metrics metrics(const std::vector<float>& y) const {
        Metrics m{ nan(), 0.f, 0.f, nan() };
        float lag = 0.f;
        if (moving_) {
            lag = xcorrLag(y);
            m.delayMs = lag * 1000.f / fs_;
        }
        double sE = 0, sE2 = 0, sRaw2 = 0, sY = 0, sY2 = 0;
        for (size_t i = a_; i < b_; ++i) {
            const double e = sampleAt(y, static_cast<double>(i) + lag) - ref_[i];  // aligned residual
            sE += e; sE2 += e * e;
            const double e0 = static_cast<double>(y[i]) - ref_[i];
            sRaw2 += e0 * e0;
            sY += y[i]; sY2 += static_cast<double>(y[i]) * y[i];
        }
        const double N = static_cast<double>(b_ - a_);
        if (moving_) {
            // noise left after removing the delay, relative to the zero-phase reference
            m.noise = static_cast<float>(std::sqrt(std::max(0.0, sE2 / N - (sE / N) * (sE / N))));
        }
        else {
            // static window: noise = plain std of the estimator output (reference-free)
            m.noise = static_cast<float>(std::sqrt(std::max(0.0, sY2 / N - (sY / N) * (sY / N))));
        }
        m.rmse = static_cast<float>(std::sqrt(sRaw2 / N));
        if (moving_ && refStd_ > 1e-6f) {
            const double sd = std::sqrt(std::max(0.0, sY2 / N - (sY / N) * (sY / N)));
            m.gain = static_cast<float>(sd / refStd_);
        }
        return m;
    }

    void addPoint(EvalCurve& c, float param, const std::vector<float>& y) const {
        const Metrics q = metrics(y);
        if (!std::isfinite(q.delayMs)) return;
        c.param.push_back(param);
        c.delayMs.push_back(q.delayMs);
        c.noiseDeg.push_back(std::max(q.noise, 1e-4f));
    }
};