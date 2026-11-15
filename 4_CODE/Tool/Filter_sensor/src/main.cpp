// MPU6050 raw-data viewer / filter design tool
// Dear ImGui + ImPlot + GLFW + OpenGL3, serial (USB CDC) input.
#include <GLFW/glfw3.h>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "implot.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <mutex>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include "frame_parser.h"
#include "sensor_data.h"
#include "serial_port.h"

// ============================================================================
// Serial reader thread
// ============================================================================
class Reader {
public:
    FrameParser parser;
    std::atomic<bool> failed{false};

    ~Reader() { stop(); }
    bool running() const { return running_; }
    const std::string& error() const { return error_; }

    bool start(const std::string& name, int baud) {
        stop();
        if (!port_.open(name, baud)) {
            error_ = port_.lastError();
            return false;
        }
        parser.reset();
        failed = false;
        running_ = true;
        th_ = std::thread([this] { loop(); });
        return true;
    }

    void stop() {
        running_ = false;
        if (th_.joinable()) th_.join();
        port_.close();
    }

    void drain(std::vector<RawFrame>& out) {
        std::lock_guard<std::mutex> lk(m_);
        out.insert(out.end(), pending_.begin(), pending_.end());
        pending_.clear();
    }

private:
    void loop() {
        std::vector<uint8_t> buf(8192);
        std::vector<RawFrame> local;
        while (running_) {
            const int n = port_.read(buf.data(), buf.size());
            if (n < 0) { failed = true; break; }
            if (n == 0) continue;
            local.clear();
            parser.feed(buf.data(), static_cast<size_t>(n), [&](const RawFrame& f) { local.push_back(f); });
            if (!local.empty()) {
                std::lock_guard<std::mutex> lk(m_);
                pending_.insert(pending_.end(), local.begin(), local.end());
            }
        }
    }

    SerialPort port_;
    std::thread th_;
    std::atomic<bool> running_{false};
    std::mutex m_;
    std::vector<RawFrame> pending_;
    std::string error_;
};

// ============================================================================
// Synthetic data source (no hardware needed)
// ============================================================================
class Demo {
public:
    void reset() { acc_ = 0; t_ = 0; }
    void generate(double wallDt, const Config& c, std::vector<RawFrame>& out) {
        acc_ += wallDt * c.fs;
        int n = static_cast<int>(acc_);
        acc_ -= n;
        n = std::min(n, static_cast<int>(c.fs * 0.25) + 1);
        std::normal_distribution<float> g(0.f, 1.f);
        const double d2r = kPi / 180.0;
        auto sat = [](float x) { return static_cast<int16_t>(std::clamp(x, -32768.f, 32767.f)); };
        for (int i = 0; i < n; ++i) {
            t_ += 1.0 / c.fs;
            const double w1 = 2 * kPi * 0.5, w2 = 2 * kPi * 0.3;
            const double phi = 20 * d2r * std::sin(w1 * t_);
            const double the = 10 * d2r * std::sin(w2 * t_ + 1.0);
            const double dphi = 20 * std::cos(w1 * t_) * w1;  // deg/s
            const double dthe = 10 * std::cos(w2 * t_ + 1.0) * w2;
            const float vib = 0.05f * static_cast<float>(std::sin(2 * kPi * 45.0 * t_));
            const float ax = static_cast<float>(-std::sin(the)) + vib + 0.02f * g(rng_);
            const float ay = static_cast<float>(std::cos(the) * std::sin(phi)) + vib + 0.02f * g(rng_);
            const float az = static_cast<float>(std::cos(the) * std::cos(phi)) + vib + 0.02f * g(rng_);
            const float gx = static_cast<float>(dphi) + 1.5f + 0.4f * g(rng_);
            const float gy = static_cast<float>(dthe) - 0.8f + 0.4f * g(rng_);
            const float gz = 0.5f + 0.4f * g(rng_);
            const float aL = accLsbPerG(c.accRange), gL = gyroLsbPerDps(c.gyroRange);
            RawFrame f;
            f.v[0] = sat(ax * aL); f.v[1] = sat(ay * aL); f.v[2] = sat(az * aL);
            f.v[3] = sat(gx * gL); f.v[4] = sat(gy * gL); f.v[5] = sat(gz * gL);
            out.push_back(f);
        }
    }

private:
    double acc_ = 0, t_ = 0;
    std::mt19937 rng_{12345};
};

// ============================================================================
// Application
// ============================================================================
static const char* kChanNames[6] = {"ax", "ay", "az", "gx", "gy", "gz"};
static const int kBauds[] = {9600, 115200, 230400, 460800, 921600, 1000000, 2000000, 3000000};

struct App {
    SensorData data;
    Reader reader;
    Demo demo;

    std::vector<std::string> ports;
    int portIdx = 0;
    int baudIdx = 1;
    bool useDemo = false, capturing = true;
    std::string status = "Disconnected";

    // display
    bool follow = true, showRaw = false, autoFitY = true;
    float windowSec = 10.f;
    int maxPts = 6000;
    double viewMin = 0, viewMax = 10;
    bool showAccAng = true, showGyroAng = true, showComp = true, showKal = true;
    float calibSeconds = 2.f;
    std::string saveMsg;
    bool dirty = false;

    // spectrum
    int specPow = 10;
    bool specDb = true;
    std::vector<float> specFreq, specRaw, specFilt;

    // stats
    uint64_t totalFrames = 0, lastFrames = 0;
    double measuredFps = 0, fpsTimer = 0;

    App() { refreshPorts(); }

    void refreshPorts() {
        ports = SerialPort::listPorts();
        portIdx = std::min(portIdx, std::max(0, static_cast<int>(ports.size()) - 1));
    }

    // ---- per-frame data intake ----------------------------------------------
    void update(double wallDt) {
        std::vector<RawFrame> frames;
        if (useDemo) {
            demo.generate(wallDt, data.cfg, frames);
        } else if (reader.running()) {
            if (reader.failed) {
                reader.stop();
                status = "Connection lost (read error)";
            } else {
                reader.drain(frames);
            }
        }
        totalFrames += frames.size();
        if (capturing)
            for (const auto& f : frames) data.append(f.v);

        fpsTimer += wallDt;
        if (fpsTimer >= 0.5) {
            measuredFps = static_cast<double>(totalFrames - lastFrames) / fpsTimer;
            lastFrames = totalFrames;
            fpsTimer = 0;
        }
    }

    // ---- UI -------------------------------------------------------------------
    void draw() {
        ImGuiViewport* vp = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(vp->WorkPos);
        ImGui::SetNextWindowSize(vp->WorkSize);
        ImGui::Begin("##root", nullptr,
                     ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                         ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoBringToFrontOnFocus);

        ImGui::BeginChild("##controls", ImVec2(390, 0), true);
        drawControls();
        ImGui::EndChild();

        if (dirty) {
            data.recomputeAll();
            dirty = false;
        }

        ImGui::SameLine();
        ImGui::BeginChild("##plots", ImVec2(0, 0), false);
        if (ImGui::BeginTabBar("##tabs")) {
            if (ImGui::BeginTabItem("Time domain")) { drawTimePlots(); ImGui::EndTabItem(); }
            if (ImGui::BeginTabItem("Spectrum"))    { drawSpectrum();  ImGui::EndTabItem(); }
            ImGui::EndTabBar();
        }
        ImGui::EndChild();
        ImGui::End();
    }

    void drawControls() {
        Config& c = data.cfg;
        ImGui::PushItemWidth(200);

        // ---- connection -------------------------------------------------------
        ImGui::SeparatorText("Connection (USB / COM)");
        ImGui::BeginDisabled(reader.running());
        if (ImGui::BeginCombo("Port", ports.empty() ? "(none)" : ports[portIdx].c_str())) {
            for (int i = 0; i < static_cast<int>(ports.size()); ++i)
                if (ImGui::Selectable(ports[i].c_str(), i == portIdx)) portIdx = i;
            ImGui::EndCombo();
        }
        ImGui::SameLine();
        if (ImGui::Button("Scan")) refreshPorts();
        if (ImGui::BeginCombo("Baud", std::to_string(kBauds[baudIdx]).c_str())) {
            for (int i = 0; i < static_cast<int>(sizeof(kBauds) / sizeof(kBauds[0])); ++i)
                if (ImGui::Selectable(std::to_string(kBauds[i]).c_str(), i == baudIdx)) baudIdx = i;
            ImGui::EndCombo();
        }
        ImGui::EndDisabled();

        if (!reader.running()) {
            ImGui::BeginDisabled(useDemo || ports.empty());
            if (ImGui::Button("Connect", ImVec2(120, 0))) {
                if (reader.start(ports[portIdx], kBauds[baudIdx])) status = "Connected: " + ports[portIdx];
                else status = "Error: " + reader.error();
            }
            ImGui::EndDisabled();
        } else if (ImGui::Button("Disconnect", ImVec2(120, 0))) {
            reader.stop();
            status = "Disconnected";
        }
        ImGui::SameLine();
        ImGui::BeginDisabled(reader.running());
        if (ImGui::Checkbox("Demo data", &useDemo)) {
            demo.reset();
            status = useDemo ? "Demo source running" : "Disconnected";
        }
        ImGui::EndDisabled();
        ImGui::TextWrapped("%s", status.c_str());

        int endian = reader.parser.bigEndian ? 1 : 0;
        if (ImGui::Combo("Byte order", &endian, "Little-endian (low byte first)\0Big-endian (high byte first)\0"))
            reader.parser.bigEndian = (endian == 1);

        if (ImGui::InputFloat("Sample rate (Hz)", &c.fs, 10.f, 100.f, "%.1f")) {
            c.fs = std::max(1.f, c.fs);
            dirty = true;
        }
        ImGui::Text("Measured: %.1f frames/s", measuredFps);
        ImGui::SameLine();
        if (ImGui::SmallButton("Use measured") && measuredFps > 1.0) {
            c.fs = static_cast<float>(std::round(measuredFps));
            dirty = true;
        }
        ImGui::Text("Frames: %llu   Bad sync: %llu", static_cast<unsigned long long>(totalFrames),
                    static_cast<unsigned long long>(reader.parser.badFrames.load()));

        // ---- sensor -------------------------------------------------------------
        ImGui::SeparatorText("Sensor");
        dirty |= ImGui::Combo("Accel range", &c.accRange, "+/-2 g\0+/-4 g\0+/-8 g\0+/-16 g\0");
        dirty |= ImGui::Combo("Gyro range", &c.gyroRange, "+/-250 dps\0+/-500 dps\0+/-1000 dps\0+/-2000 dps\0");
        dirty |= ImGui::Combo("Angle axis", &c.angleAxis, "Roll  (atan2(ay,az), gyro X)\0Pitch (atan2(-ax,|ay,az|), gyro Y)\0");
        dirty |= ImGui::Checkbox("Invert gyro sign", &c.invertGyro);
        dirty |= ImGui::DragFloat3("Gyro bias (dps)", c.gyroBias, 0.005f, -50.f, 50.f, "%.3f");
        ImGui::SliderFloat("Calib window (s)", &calibSeconds, 0.5f, 10.f, "%.1f");
        if (ImGui::Button("Calibrate gyro bias (keep still)")) {
            if (!data.calibrateGyro(calibSeconds)) status = "Not enough samples to calibrate";
        }
        ImGui::SameLine();
        if (ImGui::SmallButton("Reset")) {
            c.gyroBias[0] = c.gyroBias[1] = c.gyroBias[2] = 0.f;
            dirty = true;
        }

        // ---- filter design ------------------------------------------------------
        ImGui::SeparatorText("Channel filter");
        dirty |= ImGui::Combo("Channel", &c.chan, kChanNames, 6);
        dirty |= ImGui::Combo("Type", &c.filterType, "1st-order IIR low-pass\0Butterworth 2nd-order\0Moving average\0");
        if (c.filterType == 2) {
            dirty |= ImGui::SliderInt("Window (samples)", &c.maWindow, 1, 256);
        } else {
            dirty |= ImGui::SliderFloat("Cutoff (Hz)", &c.cutoffHz, 0.1f, std::max(1.f, c.fs * 0.5f), "%.2f",
                                        ImGuiSliderFlags_Logarithmic);
        }

        ImGui::SeparatorText("Angle estimators");
        dirty |= ImGui::SliderFloat("Comp tau (s)", &c.compTau, 0.01f, 10.f, "%.3f", ImGuiSliderFlags_Logarithmic);
        ImGui::Text("  -> alpha = %.5f", c.compTau / (c.compTau + 1.f / c.fs));
        dirty |= ImGui::SliderFloat("Kalman Q_angle", &c.kalQAngle, 1e-5f, 1.f, "%.5f", ImGuiSliderFlags_Logarithmic);
        dirty |= ImGui::SliderFloat("Kalman Q_bias", &c.kalQBias, 1e-6f, 1.f, "%.6f", ImGuiSliderFlags_Logarithmic);
        dirty |= ImGui::SliderFloat("Kalman R", &c.kalR, 1e-4f, 10.f, "%.4f", ImGuiSliderFlags_Logarithmic);

        // ---- display ------------------------------------------------------------
        ImGui::SeparatorText("Display / capture");
        ImGui::Checkbox("Capturing", &capturing);
        ImGui::SameLine();
        ImGui::Checkbox("Follow live", &follow);
        ImGui::SliderFloat("Window (s)", &windowSec, 0.5f, 120.f, "%.1f", ImGuiSliderFlags_Logarithmic);
        ImGui::Checkbox("Raw counts", &showRaw);
        ImGui::SameLine();
        ImGui::Checkbox("Auto-fit Y", &autoFitY);
        ImGui::SliderInt("Max points / plot", &maxPts, 500, 30000);
        ImGui::Checkbox("Acc angle", &showAccAng); ImGui::SameLine();
        ImGui::Checkbox("Gyro int.", &showGyroAng);
        ImGui::Checkbox("Complementary", &showComp); ImGui::SameLine();
        ImGui::Checkbox("Kalman", &showKal);

        if (ImGui::Button("Clear")) {
            data.clear();
            totalFrames = lastFrames = 0;
            reader.parser.badFrames = 0;
        }
        ImGui::SameLine();
        if (ImGui::Button("Save CSV")) {
            char name[64];
            std::time_t tt = std::time(nullptr);
            std::tm tmv{};
#ifdef _WIN32
            localtime_s(&tmv, &tt);
#else
            localtime_r(&tt, &tmv);
#endif
            std::strftime(name, sizeof(name), "mpu6050_%Y%m%d_%H%M%S.csv", &tmv);
            saveMsg = data.saveCsv(name) ? std::string("Saved ") + name : "Save failed";
        }
        if (!saveMsg.empty()) ImGui::TextWrapped("%s", saveMsg.c_str());
        ImGui::Text("Buffer: %zu samples (%.1f s)", data.size(), data.size() / double(c.fs));

        ImGui::PopItemWidth();
    }

    // ---- time-domain plots ------------------------------------------------------
    void drawTimePlots() {
        SensorData& S = data;
        const size_t n = S.size();
        const double fs = S.cfg.fs;
        const double tEnd = n ? static_cast<double>(S.t.back()) : static_cast<double>(windowSec);
        const double v0 = follow ? tEnd - windowSec : viewMin;
        const double v1 = follow ? tEnd : viewMax;

        // visible index range (+ stride decimation)
        size_t i0 = 0, i1 = n;
        if (n) {
            const double first = S.t.front();
            const double a = std::floor((v0 - first) * fs) - 1.0;
            const double b = std::ceil((v1 - first) * fs) + 2.0;
            i0 = static_cast<size_t>(std::clamp(a, 0.0, static_cast<double>(n)));
            i1 = static_cast<size_t>(std::clamp(b, static_cast<double>(i0), static_cast<double>(n)));
        }
        const size_t cnt = i1 - i0;
        const size_t step = std::max<size_t>(1, cnt / static_cast<size_t>(std::max(1, maxPts)));
        const int cntDec = static_cast<int>((cnt + step - 1) / step);

        auto line = [&](const char* label, const std::vector<float>& y) {
            if (cntDec <= 0) return;
            ImPlot::PlotLine(label, S.t.data() + i0, y.data() + i0, cntDec, 0, 0,
                             static_cast<int>(sizeof(float) * step));
        };
        const ImPlotAxisFlags yFlags = autoFitY ? (ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_RangeFit) : 0;
        auto setupPlot = [&](const char* ylabel) {
            ImPlot::SetupAxes("time (s)", ylabel, 0, yFlags);
            ImPlot::SetupAxisLimits(ImAxis_X1, v0, v1, follow ? ImPlotCond_Always : ImPlotCond_Once);
        };
        auto trackView = [&]() {
            const ImPlotRect r = ImPlot::GetPlotLimits();
            viewMin = r.X.Min;
            viewMax = r.X.Max;
            if (follow && ImPlot::IsPlotHovered() &&
                (ImGui::IsMouseDragging(ImGuiMouseButton_Left) || ImGui::GetIO().MouseWheel != 0.f))
                follow = false;  // user panned/zoomed -> stop following
        };

        if (!ImPlot::BeginSubplots("##time", 4, 1, ImVec2(-1, -1), ImPlotSubplotFlags_LinkAllX)) return;

        if (ImPlot::BeginPlot("Accelerometer")) {
            setupPlot(showRaw ? "counts" : "g");
            for (int k = 0; k < 3; ++k) line(kChanNames[k], showRaw ? S.raw[k] : S.phys[k]);
            trackView();
            ImPlot::EndPlot();
        }
        if (ImPlot::BeginPlot("Gyroscope")) {
            setupPlot(showRaw ? "counts" : "deg/s");
            for (int k = 3; k < 6; ++k) line(kChanNames[k], showRaw ? S.raw[k] : S.phys[k]);
            trackView();
            ImPlot::EndPlot();
        }
        if (ImPlot::BeginPlot(S.cfg.angleAxis == 0 ? "Angle - roll" : "Angle - pitch")) {
            setupPlot("deg");
            if (showAccAng) line("accel", S.accAng);
            if (showGyroAng) line("gyro integral", S.gyroAng);
            if (showComp) line("complementary", S.comp);
            if (showKal) line("kalman", S.kal);
            trackView();
            ImPlot::EndPlot();
        }
        const std::string title = std::string("Channel filter - ") + kChanNames[S.cfg.chan];
        if (ImPlot::BeginPlot(title.c_str())) {
            setupPlot(S.cfg.chan < 3 ? "g" : "deg/s");
            line("raw", S.phys[S.cfg.chan]);
            line("filtered", S.chanFilt);
            trackView();
            ImPlot::EndPlot();
        }
        ImPlot::EndSubplots();
    }

    // ---- spectrum -------------------------------------------------------------
    void drawSpectrum() {
        SensorData& S = data;
        const int N = 1 << specPow;
        ImGui::SetNextItemWidth(200);
        ImGui::SliderInt("FFT size = 2^n", &specPow, 7, 13);
        ImGui::SameLine();
        ImGui::Text("(%d samples, %.3f Hz/bin)", N, S.cfg.fs / N);
        ImGui::SameLine();
        ImGui::Checkbox("dB", &specDb);

        if (S.size() < static_cast<size_t>(N)) {
            ImGui::TextDisabled("Need %d samples, have %zu", N, S.size());
            return;
        }
        const auto& rawV = S.phys[S.cfg.chan];
        std::vector<float> x(rawV.end() - N, rawV.end());
        amplitudeSpectrum(x, S.cfg.fs, specFreq, specRaw);
        std::vector<float> y(S.chanFilt.end() - N, S.chanFilt.end());
        amplitudeSpectrum(y, S.cfg.fs, specFreq, specFilt);
        if (specDb) {
            for (auto& m : specRaw) m = 20.f * std::log10(std::max(m, 1e-9f));
            for (auto& m : specFilt) m = 20.f * std::log10(std::max(m, 1e-9f));
        }
        if (ImPlot::BeginPlot("Amplitude spectrum (last N samples)", ImVec2(-1, -1))) {
            ImPlot::SetupAxes("Frequency (Hz)", specDb ? "Amplitude (dB)" : "Amplitude", 0, ImPlotAxisFlags_AutoFit);
            ImPlot::SetupAxisLimits(ImAxis_X1, 0, S.cfg.fs * 0.5, ImPlotCond_Once);
            const int c = static_cast<int>(specFreq.size());
            ImPlot::PlotLine((std::string("raw ") + kChanNames[S.cfg.chan]).c_str(), specFreq.data(), specRaw.data(), c);
            ImPlot::PlotLine("filtered", specFreq.data(), specFilt.data(), c);
            ImPlot::EndPlot();
        }
    }
};

// ============================================================================
// main
// ============================================================================
static void glfwError(int code, const char* msg) { std::fprintf(stderr, "GLFW error %d: %s\n", code, msg); }

int main() {
    glfwSetErrorCallback(glfwError);
    if (!glfwInit()) return 1;

#ifdef __APPLE__
    const char* glslVersion = "#version 150";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#else
    const char* glslVersion = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
#endif

    GLFWwindow* window = glfwCreateWindow(1500, 900, "MPU6050 filter design tool", nullptr, nullptr);
    if (!window) { glfwTerminate(); return 1; }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.IniFilename = nullptr;
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glslVersion);

    {
        App app;
        auto last = std::chrono::steady_clock::now();
        while (!glfwWindowShouldClose(window)) {
            glfwPollEvents();
            if (glfwGetWindowAttrib(window, GLFW_ICONIFIED)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
                continue;
            }
            const auto now = std::chrono::steady_clock::now();
            const double dt = std::chrono::duration<double>(now - last).count();
            last = now;
            app.update(dt);

            ImGui_ImplOpenGL3_NewFrame();
            ImGui_ImplGlfw_NewFrame();
            ImGui::NewFrame();
            app.draw();
            ImGui::Render();

            int w, h;
            glfwGetFramebufferSize(window, &w, &h);
            glViewport(0, 0, w, h);
            glClearColor(0.08f, 0.08f, 0.09f, 1.f);
            glClear(GL_COLOR_BUFFER_BIT);
            ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
            glfwSwapBuffers(window);
        }
        app.reader.stop();
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
