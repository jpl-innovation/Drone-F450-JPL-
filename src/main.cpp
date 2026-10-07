// drone_telemetry: read-only MSP (MultiWii Serial Protocol v1) telemetry reader for INAV.
//
// Safety design:
//   * Only the request IDs in kAllowed are ever sent (all are read-only "get" commands).
//   * The port is opened at the requested baud and never at 1200 (that can reboot some FCs
//     into their bootloader). Over USB the baud is ignored anyway.
//   * Nothing is forwarded to the network; output goes to stdout only.
//
// Usage: drone_telemetry <serial-port> [baud=115200] [rate_hz=5]

#include <fcntl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <csignal>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <optional>
#include <string>
#include <vector>

namespace {

// ---- MSP v1 ----------------------------------------------------------------
constexpr uint8_t MSP_API_VERSION = 1;
constexpr uint8_t MSP_STATUS      = 101;
constexpr uint8_t MSP_ATTITUDE    = 108;
constexpr uint8_t MSP_ANALOG      = 110;

constexpr std::array<uint8_t, 4> kAllowed = {MSP_API_VERSION, MSP_STATUS, MSP_ATTITUDE, MSP_ANALOG};

bool allowed(uint8_t cmd) {
  return std::find(kAllowed.begin(), kAllowed.end(), cmd) != kAllowed.end();
}

struct Frame {
  uint8_t cmd = 0;
  bool error = false;  // '!' reply
  std::vector<uint8_t> payload;
};

std::vector<uint8_t> buildRequest(uint8_t cmd) {
  // "$M<" size cmd checksum ; checksum = size ^ cmd ^ payload (no payload here)
  return {'$', 'M', '<', 0, cmd, static_cast<uint8_t>(0 ^ cmd)};
}

class Parser {
 public:
  // Feed one byte; returns a frame when one completes with a valid checksum.
  std::optional<Frame> feed(uint8_t b) {
    switch (st_) {
      case St::Idle:
        if (b == '$') st_ = St::M;
        break;
      case St::M:
        st_ = (b == 'M') ? St::Dir : St::Idle;
        break;
      case St::Dir:
        if (b == '>' || b == '!') {
          f_ = Frame{};
          f_.error = (b == '!');
          st_ = St::Size;
        } else {
          st_ = (b == '$') ? St::M : St::Idle;
        }
        break;
      case St::Size:
        size_ = b;
        crc_ = b;
        st_ = St::Cmd;
        break;
      case St::Cmd:
        f_.cmd = b;
        crc_ ^= b;
        f_.payload.clear();
        st_ = size_ ? St::Payload : St::Crc;
        break;
      case St::Payload:
        f_.payload.push_back(b);
        crc_ ^= b;
        if (f_.payload.size() == size_) st_ = St::Crc;
        break;
      case St::Crc:
        st_ = St::Idle;
        if (b == crc_) return f_;
        ++badCrc;
        break;
    }
    return std::nullopt;
  }
  unsigned badCrc = 0;

 private:
  enum class St { Idle, M, Dir, Size, Cmd, Payload, Crc };
  St st_ = St::Idle;
  uint8_t size_ = 0;
  uint8_t crc_ = 0;
  Frame f_;
};

uint16_t u16(const std::vector<uint8_t>& p, size_t i) { return static_cast<uint16_t>(p[i] | (p[i + 1] << 8)); }
int16_t i16(const std::vector<uint8_t>& p, size_t i) { return static_cast<int16_t>(u16(p, i)); }
uint32_t u32(const std::vector<uint8_t>& p, size_t i) {
  return static_cast<uint32_t>(p[i]) | (static_cast<uint32_t>(p[i + 1]) << 8) |
         (static_cast<uint32_t>(p[i + 2]) << 16) | (static_cast<uint32_t>(p[i + 3]) << 24);
}

// ---- Telemetry snapshot -----------------------------------------------------
struct Telemetry {
  bool haveAtt = false, haveAnalog = false, haveStatus = false;
  double roll = 0, pitch = 0;   // degrees
  int yaw = 0;                  // degrees
  double vbat = 0;              // volts
  double amps = 0;              // amps
  unsigned mah = 0, rssi = 0;   // rssi 0..1023
  unsigned cycleUs = 0;
  uint32_t modeFlags = 0;
  int apiMajor = 0, apiMinor = 0;
};

void apply(const Frame& f, Telemetry& t) {
  const auto& p = f.payload;
  switch (f.cmd) {
    case MSP_API_VERSION:  // protocol, api major, api minor
      if (p.size() >= 3) { t.apiMajor = p[1]; t.apiMinor = p[2]; }
      break;
    case MSP_ATTITUDE:  // roll(i16, 0.1deg), pitch(i16, 0.1deg), yaw(i16, deg)
      if (p.size() >= 6) {
        t.roll = i16(p, 0) / 10.0;
        t.pitch = i16(p, 2) / 10.0;
        t.yaw = i16(p, 4);
        t.haveAtt = true;
      }
      break;
    case MSP_ANALOG:  // vbat(u8, 0.1V), mAh(u16), rssi(u16), amps(i16, 0.01A)
      if (p.size() >= 7) {
        t.vbat = p[0] / 10.0;
        t.mah = u16(p, 1);
        t.rssi = u16(p, 3);
        t.amps = i16(p, 5) / 100.0;
        t.haveAnalog = true;
      }
      break;
    case MSP_STATUS:  // cycleTime(u16), i2cErr(u16), sensors(u16), flightModeFlags(u32), ...
      if (p.size() >= 10) {
        t.cycleUs = u16(p, 0);
        t.modeFlags = u32(p, 6);
        t.haveStatus = true;
      }
      break;
    default:
      break;
  }
}

// ---- Serial -----------------------------------------------------------------
speed_t toSpeed(int baud) {
  switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    default: return 0;
  }
}

int openPort(const char* path, int baud) {
  speed_t spd = toSpeed(baud);
  if (spd == 0) {
    std::fprintf(stderr, "unsupported baud %d (1200 is deliberately not supported)\n", baud);
    return -1;
  }
  int fd = ::open(path, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) { std::perror("open"); return -1; }

  termios tio{};
  if (tcgetattr(fd, &tio) != 0) { std::perror("tcgetattr"); ::close(fd); return -1; }
  cfmakeraw(&tio);                       // raw: no echo, no line editing, no CR/LF mangling
  tio.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
  tio.c_cflag |= CS8 | CLOCAL | CREAD;   // 8N1, no flow control
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;
  cfsetispeed(&tio, spd);
  cfsetospeed(&tio, spd);
  if (tcsetattr(fd, TCSANOW, &tio) != 0) { std::perror("tcsetattr"); ::close(fd); return -1; }
  tcflush(fd, TCIOFLUSH);
  return fd;
}

volatile std::sig_atomic_t g_stop = 0;
void onSignal(int) { g_stop = 1; }

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::fprintf(stderr, "usage: %s <serial-port> [baud=115200] [rate_hz=5]\n", argv[0]);
    return 2;
  }
  const int baud = argc > 2 ? std::atoi(argv[2]) : 115200;
  const int rate = std::clamp(argc > 3 ? std::atoi(argv[3]) : 5, 1, 20);

  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);

  const int fd = openPort(argv[1], baud);
  if (fd < 0) return 1;

  using clk = std::chrono::steady_clock;
  const auto period = std::chrono::milliseconds(1000 / rate);
  auto next = clk::now();
  Parser parser;
  Telemetry t;
  auto lastReply = clk::now();
  bool warned = false;

  // One poll per period: send the allow-listed requests, then read replies until the next tick.
  while (!g_stop) {
    const auto now = clk::now();
    if (now >= next) {
      next += period;
      for (uint8_t cmd : {MSP_ATTITUDE, MSP_ANALOG, MSP_STATUS}) {
        if (!allowed(cmd)) continue;  // belt and braces
        const auto req = buildRequest(cmd);
        if (::write(fd, req.data(), req.size()) < 0) { std::perror("write"); g_stop = 1; break; }
      }
      if (t.haveAtt || t.haveAnalog || t.haveStatus) {
        std::printf("roll %6.1f  pitch %6.1f  yaw %4d | %4.1f V  %5.2f A  %4u mAh  rssi %4u | loop %4u us  modes 0x%08x\n",
                    t.roll, t.pitch, t.yaw, t.vbat, t.amps, t.mah, t.rssi, t.cycleUs, t.modeFlags);
        std::fflush(stdout);
      }
    }

    fd_set rf;
    FD_ZERO(&rf);
    FD_SET(fd, &rf);
    timeval tv{0, 20000};  // 20 ms
    if (::select(fd + 1, &rf, nullptr, nullptr, &tv) > 0) {
      uint8_t buf[256];
      const ssize_t n = ::read(fd, buf, sizeof buf);
      if (n < 0 && errno != EAGAIN && errno != EINTR) { std::perror("read"); break; }
      for (ssize_t i = 0; i < n; ++i) {
        if (auto f = parser.feed(buf[i])) {
          if (f->error) continue;
          apply(*f, t);
          lastReply = clk::now();
          warned = false;
        }
      }
    }
    if (!warned && clk::now() - lastReply > std::chrono::seconds(3)) {
      std::fprintf(stderr, "no valid MSP reply for 3 s (wrong port? Configurator holding the USB port?)\n");
      warned = true;
    }
  }
  ::close(fd);
  return 0;
}
