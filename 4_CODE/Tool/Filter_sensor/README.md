# MPU6050 filter design tool

PC app (C++17, CMake, Dear ImGui + ImPlot + GLFW + OpenGL3) that reads raw MPU6050
data from a USB virtual COM port and plots it so you can tune sensor filters.

## Frame format (15 bytes)

| byte    | value                                   |
|---------|-----------------------------------------|
| 0       | `0xFC` header                           |
| 1       | `12` length                             |
| 2..13   | `ax ay az gx gy gz`, int16 each (raw)   |
| 14      | `0xFF` end                              |

Byte order of each int16 is selectable in the UI (default: low byte first).
The parser re-synchronises automatically if bytes are lost or a payload byte equals `0xFC`.

Firmware side (little-endian example):

```c
int16_t d[6] = {ax, ay, az, gx, gy, gz};
uint8_t pkt[15];
pkt[0] = 0xFC;
pkt[1] = 12;
for (int i = 0; i < 6; i++) {
    pkt[2 + 2*i] = (uint8_t)(d[i] & 0xFF);
    pkt[3 + 2*i] = (uint8_t)((d[i] >> 8) & 0xFF);
}
pkt[14] = 0xFF;
CDC_Transmit_FS(pkt, 15);   // or UART
```

## Build

Needs CMake >= 3.16, a C++17 compiler and internet access on the first configure
(GLFW, ImGui, ImPlot are fetched with FetchContent).

```bash
cd tool
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
```

- Windows (Visual Studio): `build\Release\mpu6050_tool.exe`
- Linux: `sudo apt install build-essential cmake libgl1-mesa-dev xorg-dev` first;
  add yourself to the `dialout` group to open `/dev/ttyACM*`.

## Usage

1. Choose the port, press **Connect** (or tick **Demo data** to test without hardware).
2. Set **Sample rate (Hz)** to the real rate of your frames. The frame has no timestamp, so
   time axis and filter `dt` come from this value; **Measured** shows what is actually arriving.
3. Set accel/gyro range to match the MPU6050 configuration.
4. Keep the sensor still and press **Calibrate gyro bias**.
5. Untick **Capturing** to freeze a recording, then tweak filter parameters: every plot is
   recomputed from the same raw data, so you can compare settings fairly.

Plots (time axes linked):

- Accelerometer and gyroscope (physical units or raw counts)
- Angle: accelerometer angle, integrated gyro, complementary filter, Kalman filter
- Selected channel: raw vs filtered (1st-order IIR, Butterworth 2nd order, moving average)
- Spectrum tab: FFT of the selected channel, raw vs filtered, to pick the cutoff

**Save CSV** writes the whole buffer (raw, converted values and all filter outputs).

Notes: angle wrap-around at +/-180 deg is not handled; buffer keeps the last 300k samples.
