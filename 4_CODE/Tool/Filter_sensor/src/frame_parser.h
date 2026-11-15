#pragma once
// Frame format (15 bytes):
//   [0]      header = 0xFC
//   [1]      length = 12
//   [2..13]  ax, ay, az, gx, gy, gz  (6 x int16, raw MPU6050 counts)
//   [14]     end    = 0xFF
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace proto {
constexpr uint8_t kHeader = 0xFC;
constexpr uint8_t kLength = 12;
constexpr uint8_t kEnd = 0xFF;
constexpr size_t kFrameSize = 1 + 1 + kLength + 1;  // 15
}  // namespace proto

struct RawFrame {
    int16_t v[6];  // ax ay az gx gy gz
};

class FrameParser {
public:
    std::atomic<bool> bigEndian{false};  // false: low byte first
    std::atomic<uint64_t> goodFrames{0};
    std::atomic<uint64_t> badFrames{0};

    void reset() {
        buf_.clear();
        goodFrames = 0;
        badFrames = 0;
    }

    // Feed raw bytes; onFrame(const RawFrame&) is called for each valid frame.
    // A header byte that is not followed by a valid frame is skipped one byte at
    // a time, so the parser re-synchronises even if payload bytes equal 0xFC.
    template <typename Fn>
    void feed(const uint8_t* data, size_t n, Fn&& onFrame) {
        buf_.insert(buf_.end(), data, data + n);
        size_t i = 0;
        for (;;) {
            while (i < buf_.size() && buf_[i] != proto::kHeader) ++i;
            if (buf_.size() - i < proto::kFrameSize) break;

            const uint8_t* p = &buf_[i];
            if (p[1] == proto::kLength && p[proto::kFrameSize - 1] == proto::kEnd) {
                const bool be = bigEndian.load(std::memory_order_relaxed);
                RawFrame f;
                for (int k = 0; k < 6; ++k) {
                    const uint8_t a = p[2 + 2 * k];
                    const uint8_t b = p[3 + 2 * k];
                    const uint16_t u = be ? uint16_t((a << 8) | b) : uint16_t((b << 8) | a);
                    f.v[k] = static_cast<int16_t>(u);
                }
                ++goodFrames;
                i += proto::kFrameSize;
                onFrame(f);
            } else {
                ++badFrames;
                ++i;
            }
        }
        buf_.erase(buf_.begin(), buf_.begin() + static_cast<std::ptrdiff_t>(i));
    }

private:
    std::vector<uint8_t> buf_;
};
