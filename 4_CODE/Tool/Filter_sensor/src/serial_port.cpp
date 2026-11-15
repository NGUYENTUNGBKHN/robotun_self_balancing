#include "serial_port.h"

#include <algorithm>
#include <cstring>

#ifdef _WIN32
// ============================================================================
// Windows
// ============================================================================
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>

bool SerialPort::open(const std::string& name, int baud) {
    close();
    std::string path = name;
    if (path.rfind("\\\\.\\", 0) != 0) path = "\\\\.\\" + path;  // needed for COM10+

    HANDLE h = CreateFileA(path.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr,
                           OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
    if (h == INVALID_HANDLE_VALUE) {
        err_ = "CreateFile(" + name + ") failed, error " + std::to_string(GetLastError());
        return false;
    }
    SetupComm(h, 1 << 16, 1 << 12);

    DCB dcb;
    ZeroMemory(&dcb, sizeof(dcb));
    dcb.DCBlength = sizeof(dcb);
    if (!GetCommState(h, &dcb)) {
        err_ = "GetCommState failed";
        CloseHandle(h);
        return false;
    }
    dcb.BaudRate = static_cast<DWORD>(baud);
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fBinary = TRUE;
    dcb.fParity = FALSE;
    dcb.fOutxCtsFlow = FALSE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fDtrControl = DTR_CONTROL_ENABLE;  // many CDC devices only send when DTR is set
    dcb.fRtsControl = RTS_CONTROL_ENABLE;
    dcb.fOutX = FALSE;
    dcb.fInX = FALSE;
    dcb.fAbortOnError = FALSE;
    if (!SetCommState(h, &dcb)) {
        err_ = "SetCommState failed, error " + std::to_string(GetLastError());
        CloseHandle(h);
        return false;
    }

    // Return immediately if data is available, otherwise wait up to 20 ms.
    COMMTIMEOUTS to;
    ZeroMemory(&to, sizeof(to));
    to.ReadIntervalTimeout = MAXDWORD;
    to.ReadTotalTimeoutMultiplier = MAXDWORD;
    to.ReadTotalTimeoutConstant = 20;
    to.WriteTotalTimeoutConstant = 1000;
    SetCommTimeouts(h, &to);
    PurgeComm(h, PURGE_RXCLEAR | PURGE_TXCLEAR);

    h_ = reinterpret_cast<intptr_t>(h);
    err_.clear();
    return true;
}

void SerialPort::close() {
    if (h_ != -1) {
        CloseHandle(reinterpret_cast<HANDLE>(h_));
        h_ = -1;
    }
}

int SerialPort::read(uint8_t* buf, size_t maxLen) {
    if (h_ == -1) return -1;
    DWORD got = 0;
    if (!ReadFile(reinterpret_cast<HANDLE>(h_), buf, static_cast<DWORD>(maxLen), &got, nullptr))
        return -1;
    return static_cast<int>(got);
}

std::vector<std::string> SerialPort::listPorts() {
    std::vector<std::string> ports;
    std::vector<char> buf(1 << 16);
    DWORD n = 0;
    for (;;) {
        n = QueryDosDeviceA(nullptr, buf.data(), static_cast<DWORD>(buf.size()));
        if (n == 0 && GetLastError() == ERROR_INSUFFICIENT_BUFFER) {
            buf.resize(buf.size() * 2);
            continue;
        }
        break;
    }
    for (const char* p = buf.data(); n && *p && p < buf.data() + n; p += std::strlen(p) + 1) {
        if (std::strncmp(p, "COM", 3) == 0) ports.emplace_back(p);
    }
    std::sort(ports.begin(), ports.end(), [](const std::string& a, const std::string& b) {
        return a.size() != b.size() ? a.size() < b.size() : a < b;
    });
    return ports;
}

#else
// ============================================================================
// Linux / macOS
// ============================================================================
#include <errno.h>
#include <fcntl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include <filesystem>

static bool toSpeed(int baud, speed_t& s) {
    switch (baud) {
        case 9600: s = B9600; return true;
        case 19200: s = B19200; return true;
        case 38400: s = B38400; return true;
        case 57600: s = B57600; return true;
        case 115200: s = B115200; return true;
        case 230400: s = B230400; return true;
#ifdef B460800
        case 460800: s = B460800; return true;
#endif
#ifdef B921600
        case 921600: s = B921600; return true;
#endif
#ifdef B1000000
        case 1000000: s = B1000000; return true;
#endif
#ifdef B2000000
        case 2000000: s = B2000000; return true;
#endif
#ifdef B3000000
        case 3000000: s = B3000000; return true;
#endif
        default: return false;
    }
}

bool SerialPort::open(const std::string& name, int baud) {
    close();
    speed_t speed;
    if (!toSpeed(baud, speed)) {
        err_ = "Unsupported baud rate " + std::to_string(baud);
        return false;
    }
    int fd = ::open(name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        err_ = "open(" + name + ") failed: " + std::strerror(errno);
        return false;
    }
    termios tio;
    if (tcgetattr(fd, &tio) != 0) {
        err_ = "tcgetattr failed";
        ::close(fd);
        return false;
    }
    cfmakeraw(&tio);
    cfsetispeed(&tio, speed);
    cfsetospeed(&tio, speed);
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;
    if (tcsetattr(fd, TCSANOW, &tio) != 0) {
        err_ = "tcsetattr failed";
        ::close(fd);
        return false;
    }
    tcflush(fd, TCIOFLUSH);
    h_ = fd;
    err_.clear();
    return true;
}

void SerialPort::close() {
    if (h_ != -1) {
        ::close(static_cast<int>(h_));
        h_ = -1;
    }
}

int SerialPort::read(uint8_t* buf, size_t maxLen) {
    if (h_ == -1) return -1;
    const int fd = static_cast<int>(h_);
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(fd, &rfds);
    timeval tv{0, 20000};
    int r = select(fd + 1, &rfds, nullptr, nullptr, &tv);
    if (r < 0) return errno == EINTR ? 0 : -1;
    if (r == 0) return 0;
    ssize_t n = ::read(fd, buf, maxLen);
    if (n < 0) return (errno == EAGAIN || errno == EINTR) ? 0 : -1;
    if (n == 0) return -1;  // device unplugged
    return static_cast<int>(n);
}

std::vector<std::string> SerialPort::listPorts() {
    namespace fs = std::filesystem;
    std::vector<std::string> ports;
    std::error_code ec;
    for (const auto& e : fs::directory_iterator("/dev", ec)) {
        const std::string n = e.path().filename().string();
        if (n.rfind("ttyUSB", 0) == 0 || n.rfind("ttyACM", 0) == 0 ||
            n.rfind("cu.usb", 0) == 0 || n.rfind("cu.SLAB", 0) == 0 || n.rfind("cu.wch", 0) == 0)
            ports.push_back("/dev/" + n);
    }
    std::sort(ports.begin(), ports.end());
    return ports;
}
#endif
