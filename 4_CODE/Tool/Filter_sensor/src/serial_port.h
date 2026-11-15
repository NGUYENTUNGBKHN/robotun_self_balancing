#pragma once
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

// Minimal cross-platform serial port (USB CDC / USB-UART virtual COM port).
class SerialPort {
public:
    SerialPort() = default;
    ~SerialPort() { close(); }
    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    bool open(const std::string& name, int baud);
    void close();
    bool isOpen() const { return h_ != -1; }

    // Waits up to ~20 ms. Returns bytes read (0 on timeout) or -1 on error/disconnect.
    int read(uint8_t* buf, size_t maxLen);

    const std::string& lastError() const { return err_; }
    static std::vector<std::string> listPorts();

private:
    intptr_t h_ = -1;
    std::string err_;
};
