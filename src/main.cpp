// drone_telemetry v2: read-only MSP telemetry reader for INAV (JPL Innovation F450).
//
// Safety design:
//   * Only the request IDs in kAllowed are ever sent. Every one of them is a read-only
//     "get" command: nothing here can change a setting, arm, or move a motor.
//   * The port is opened at the requested baud and never at 1200 (that can reboot some FCs
//     into their bootloader). Over USB the baud is ignored anyway.
//   * Output: a human-readable line on stdout (the journal) and, for the dashboard, JSON
//     datagrams sent ONLY to a loopback address (127.x.x.x). Any other address is refused,
//     so nothing leaves the Pi from this program.
//
// Speed: requests are self-paced. The next request goes out as soon as the FC answers the
// previous one (or after a 50 ms timeout), capped at --max-req-hz. Attitude gets half of all
// slots; battery, motors, RC, IMU, altitude and status share the rest.
//
// Accuracy: battery data comes from INAV's MSP2_INAV_ANALOG (0.01 V, 0.01 A, W, Wh, %, cells).
// If the FC answers that with an error, the reader falls back to MSP_ANALOG (0.1 V).
//
// Usage: drone_telemetry <serial-port> [baud=115200] [print_hz=5]
//                        [--udp=127.0.0.1:14650 | --udp=off] [--max-req-hz=120]

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace {

using clk = std::chrono::steady_clock;

// ---- MSP command IDs (all read-only) ---------------------------------------
constexpr uint16_t MSP_API_VERSION  = 1;
constexpr uint16_t MSP_STATUS       = 101;
constexpr uint16_t MSP_RAW_IMU      = 102;
constexpr uint16_t MSP_MOTOR        = 104;
constexpr uint16_t MSP_RC           = 105;
constexpr uint16_t MSP_ATTITUDE     = 108;
constexpr uint16_t MSP_ALTITUDE     = 109;
constexpr uint16_t MSP_ANALOG       = 110;
constexpr uint16_t MSP_BOXNAMES     = 116;
constexpr uint16_t MSP2_INAV_STATUS = 0x2000;
constexpr uint16_t MSP2_INAV_ANALOG = 0x2002;

constexpr std::array<uint16_t, 11> kAllowed = {
    MSP_API_VERSION, MSP_STATUS, MSP_RAW_IMU, MSP_MOTOR, MSP_RC, MSP_ATTITUDE,
    MSP_ALTITUDE, MSP_ANALOG, MSP_BOXNAMES, MSP2_INAV_STATUS, MSP2_INAV_ANALOG};

bool allowed(uint16_t cmd) {
  return std::find(kAllowed.begin(), kAllowed.end(), cmd) != kAllowed.end();
}

// Round-robin of polled requests. Attitude takes every other slot.
constexpr std::array<uint16_t, 12> kCycle = {
    MSP_ATTITUDE, MSP_RC,       MSP_ATTITUDE, MSP_MOTOR,
    MSP_ATTITUDE, MSP2_INAV_ANALOG, MSP_ATTITUDE, MSP_RAW_IMU,
    MSP_ATTITUDE, MSP_ALTITUDE, MSP_ATTITUDE, MSP2_INAV_STATUS};

// ---- MSP v2 framing ----------------------------------------------------------
uint8_t crc8(uint8_t crc, uint8_t a) {  // CRC-8/DVB-S2, as used by MSP v2
  crc ^= a;
  for (int i = 0; i < 8; ++i) crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0xD5) : static_cast<uint8_t>(crc << 1);
  return crc;
}

std::vector<uint8_t> buildRequest(uint16_t cmd) {
  // "$X<" flag cmd(u16) size(u16) [payload] crc ; crc over flag..payload. No payload here.
  std::vector<uint8_t> f = {'$', 'X', '<', 0, static_cast<uint8_t>(cmd & 0xFF), static_cast<uint8_t>(cmd >> 8), 0, 0};
  uint8_t c = 0;
  for (size_t i = 3; i < f.size(); ++i) c = crc8(c, f[i]);
  f.push_back(c);
  return f;
}

struct Frame {
  uint16_t cmd = 0;
  bool error = false;  // '!' reply: FC does not support this command
  std::vector<uint8_t> payload;
};

class Parser {  // accepts MSP v1 ($M) and v2 ($X) replies
 public:
  std::optional<Frame> feed(uint8_t b) {
    switch (st_) {
      case St::Idle:
        if (b == '$') st_ = St::Proto;
        break;
      case St::Proto:
        if (b == 'M' || b == 'X') { v2_ = (b == 'X'); st_ = St::Dir; }
        else st_ = (b == '$') ? St::Proto : St::Idle;
        break;
      case St::Dir:
        if (b == '>' || b == '!') {
          f_ = Frame{};
          f_.error = (b == '!');
          size_ = 0;
          st_ = v2_ ? St::Flag : St::Size1;
        } else {
          st_ = (b == '$') ? St::Proto : St::Idle;
        }
        break;
      case St::Size1: size_ = b; crc_ = b; st_ = St::Cmd1; break;
      case St::Cmd1:
        f_.cmd = b; crc_ ^= b;
        st_ = size_ ? St::Payload : St::Crc;
        break;
      case St::Flag: crc_ = crc8(0, b); st_ = St::CmdLo; break;
      case St::CmdLo: f_.cmd = b; crc_ = crc8(crc_, b); st_ = St::CmdHi; break;
      case St::CmdHi: f_.cmd |= static_cast<uint16_t>(b) << 8; crc_ = crc8(crc_, b); st_ = St::SizeLo; break;
      case St::SizeLo: size_ = b; crc_ = crc8(crc_, b); st_ = St::SizeHi; break;
      case St::SizeHi:
        size_ |= static_cast<size_t>(b) << 8; crc_ = crc8(crc_, b);
        if (size_ > 4096) { ++badFrames; st_ = St::Idle; break; }
        st_ = size_ ? St::Payload : St::Crc;
        break;
      case St::Payload:
        f_.payload.push_back(b);
        crc_ = v2_ ? crc8(crc_, b) : static_cast<uint8_t>(crc_ ^ b);
        if (f_.payload.size() == size_) st_ = St::Crc;
        break;
      case St::Crc:
        st_ = St::Idle;
        if (b == crc_) return f_;
        ++badFrames;
        break;
    }
    return std::nullopt;
  }
  unsigned badFrames = 0;

 private:
  enum class St { Idle, Proto, Dir, Size1, Cmd1, Flag, CmdLo, CmdHi, SizeLo, SizeHi, Payload, Crc };
  St st_ = St::Idle;
  bool v2_ = false;
  size_t size_ = 0;
  uint8_t crc_ = 0;
  Frame f_;
};

uint16_t u16(const std::vector<uint8_t>& p, size_t i) { return static_cast<uint16_t>(p[i] | (p[i + 1] << 8)); }
int16_t i16(const std::vector<uint8_t>& p, size_t i) { return static_cast<int16_t>(u16(p, i)); }
uint32_t u32(const std::vector<uint8_t>& p, size_t i) {
  return static_cast<uint32_t>(p[i]) | (static_cast<uint32_t>(p[i + 1]) << 8) |
         (static_cast<uint32_t>(p[i + 2]) << 16) | (static_cast<uint32_t>(p[i + 3]) << 24);
}
int32_t i32(const std::vector<uint8_t>& p, size_t i) { return static_cast<int32_t>(u32(p, i)); }

// ---- Telemetry snapshot -------------------------------------------------------
struct Telemetry {
  clk::time_point tAtt{}, tBat{}, tStatus{}, tMotor{}, tRc{}, tImu{}, tAlt{};
  double roll = 0, pitch = 0; int yaw = 0;
  double vbat = 0, amps = 0, watts = 0; uint32_t mah = 0, mwh = 0, remainMah = 0;
  int battPct = -1, cells = 0, battState = -1; unsigned rssi = 0; bool batV2 = false;
  unsigned cycleUs = 0, i2cErr = 0, sensors = 0, cpuLoad = 0; int profile = -1;
  uint32_t armingFlags = 0; uint64_t boxFlags = 0; bool statusV2 = false;
  std::vector<uint16_t> motors, rc;
  double acc[3]{}, gyro[3]{}; int mag[3]{};
  double altM = 0, varioMs = 0, baroAltM = 0; bool haveBaro = false;
  std::vector<std::string> boxNames;
  int apiMajor = 0, apiMinor = 0;
  bool noV2Analog = false, noV2Status = false;
  bool any() const { return tAtt != clk::time_point{} || tBat != clk::time_point{} || tStatus != clk::time_point{}; }
};

void apply(const Frame& f, Telemetry& t, clk::time_point now) {
  const auto& p = f.payload;
  if (f.error) {  // unsupported command: fall back to the MSP v1 equivalent
    if (f.cmd == MSP2_INAV_ANALOG) t.noV2Analog = true;
    if (f.cmd == MSP2_INAV_STATUS) t.noV2Status = true;
    return;
  }
  switch (f.cmd) {
    case MSP_API_VERSION:
      if (p.size() >= 3) { t.apiMajor = p[1]; t.apiMinor = p[2]; }
      break;
    case MSP_ATTITUDE:  // roll, pitch (i16, 0.1 deg), yaw (i16, deg)
      if (p.size() >= 6) { t.roll = i16(p, 0) / 10.0; t.pitch = i16(p, 2) / 10.0; t.yaw = i16(p, 4); t.tAtt = now; }
      break;
    case MSP2_INAV_ANALOG:  // flags, V(0.01), A(0.01), W(0.01), mAh, mWh, remaining mAh, %, rssi
      if (p.size() >= 22) {
        t.cells = p[0] >> 4;
        t.battState = (p[0] >> 2) & 0x3;
        t.vbat = u16(p, 1) / 100.0;
        t.amps = i16(p, 3) / 100.0;
        t.watts = i32(p, 5) / 100.0;
        t.mah = u32(p, 9);
        t.mwh = u32(p, 13);
        t.remainMah = u32(p, 17);
        t.battPct = p[21];
        if (p.size() >= 24) t.rssi = u16(p, 22);
        t.batV2 = true;
        t.tBat = now;
      }
      break;
    case MSP_ANALOG:  // vbat(u8, 0.1V), mAh(u16), rssi(u16), amps(i16, 0.01A)
      if (p.size() >= 7) {
        t.vbat = p[0] / 10.0; t.mah = u16(p, 1); t.rssi = u16(p, 3); t.amps = i16(p, 5) / 100.0;
        t.watts = t.vbat * t.amps; t.batV2 = false; t.tBat = now;
      }
      break;
    case MSP2_INAV_STATUS:  // cycle, i2c, sensors, load%, profile, armingFlags(u32), boxFlags(...), mixer
      if (p.size() >= 13) {
        t.cycleUs = u16(p, 0); t.i2cErr = u16(p, 2); t.sensors = u16(p, 4); t.cpuLoad = u16(p, 6);
        t.profile = p[8]; t.armingFlags = u32(p, 9);
        const size_t n = std::min<size_t>(p.size() > 14 ? p.size() - 14 : p.size() - 13, 8);
        uint64_t b = 0;
        for (size_t i = 0; i < n; ++i) b |= static_cast<uint64_t>(p[13 + i]) << (8 * i);
        t.boxFlags = b; t.statusV2 = true; t.tStatus = now;
      }
      break;
    case MSP_STATUS:  // cycleTime, i2cErr, sensors, flightModeFlags(u32)
      if (p.size() >= 10) {
        t.cycleUs = u16(p, 0); t.i2cErr = u16(p, 2); t.sensors = u16(p, 4); t.boxFlags = u32(p, 6);
        t.statusV2 = false; t.tStatus = now;
      }
      break;
    case MSP_MOTOR:
      t.motors.clear();
      for (size_t i = 0; i + 1 < p.size() && t.motors.size() < 8; i += 2) t.motors.push_back(u16(p, i));
      t.tMotor = now;
      break;
    case MSP_RC:
      t.rc.clear();
      for (size_t i = 0; i + 1 < p.size() && t.rc.size() < 34; i += 2) t.rc.push_back(u16(p, i));
      t.tRc = now;
      break;
    case MSP_RAW_IMU:  // acc (g*512), gyro (deg/s), mag (raw)
      if (p.size() >= 18) {
        for (int i = 0; i < 3; ++i) {
          t.acc[i] = i16(p, 2 * i) / 512.0;
          t.gyro[i] = i16(p, 6 + 2 * i);
          t.mag[i] = i16(p, 12 + 2 * i);
        }
        t.tImu = now;
      }
      break;
    case MSP_ALTITUDE:  // estimated alt (i32 cm), vario (i16 cm/s), baro alt (i32 cm)
      if (p.size() >= 6) {
        t.altM = i32(p, 0) / 100.0; t.varioMs = i16(p, 4) / 100.0;
        if (p.size() >= 10) { t.baroAltM = i32(p, 6) / 100.0; t.haveBaro = true; }
        t.tAlt = now;
      }
      break;
    case MSP_BOXNAMES: {
      t.boxNames.clear();
      std::string cur;
      for (uint8_t c : p) {
        if (c == ';') { t.boxNames.push_back(cur); cur.clear(); }
        else if (c >= 32 && c < 127) cur.push_back(static_cast<char>(c));
      }
      if (!cur.empty()) t.boxNames.push_back(cur);
      break;
    }
    default:
      break;
  }
}

// ---- JSON helpers ---------------------------------------------------------------
void appendf(std::string& s, const char* fmt, ...) {
  char buf[512];
  va_list ap;
  va_start(ap, fmt);
  const int n = std::vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  if (n > 0) s.append(buf, std::min<size_t>(static_cast<size_t>(n), sizeof buf - 1));
}

long ageMs(clk::time_point t, clk::time_point now) {
  if (t == clk::time_point{}) return -1;
  return static_cast<long>(std::chrono::duration_cast<std::chrono::milliseconds>(now - t).count());
}

std::string jsonEscape(const std::string& in) {
  std::string o;
  for (char c : in) {
    if (c == '"' || c == '\\') { o.push_back('\\'); o.push_back(c); }
    else if (static_cast<unsigned char>(c) >= 32) o.push_back(c);
  }
  return o;
}

struct Rates { double att = 0, bat = 0, motor = 0, rc = 0, imu = 0, alt = 0, status = 0, req = 0, timeouts = 0; };

std::string buildJson(const Telemetry& t, const Rates& r, unsigned long seq, unsigned bad, clk::time_point now) {
  std::string s;
  s.reserve(1400);
  appendf(s, "{\"type\":\"t\",\"seq\":%lu,", seq);
  appendf(s, "\"att\":{\"roll\":%.1f,\"pitch\":%.1f,\"yaw\":%d,\"age\":%ld},", t.roll, t.pitch, t.yaw, ageMs(t.tAtt, now));
  appendf(s, "\"bat\":{\"v\":%.2f,\"a\":%.2f,\"w\":%.2f,\"mah\":%u,\"mwh\":%u,\"remain\":%u,\"pct\":%d,\"cells\":%d,\"state\":%d,\"src\":\"%s\",\"age\":%ld},",
          t.vbat, t.amps, t.watts, t.mah, t.mwh, t.remainMah, t.battPct, t.cells, t.battState, t.batV2 ? "v2" : "v1", ageMs(t.tBat, now));
  appendf(s, "\"rssi\":%u,", t.rssi);
  appendf(s, "\"status\":{\"cycle\":%u,\"i2c\":%u,\"sensors\":%u,\"load\":%u,\"profile\":%d,\"arming\":%u,\"modes_lo\":%u,\"modes_hi\":%u,\"src\":\"%s\",\"age\":%ld},",
          t.cycleUs, t.i2cErr, t.sensors, t.cpuLoad, t.profile, t.armingFlags,
          static_cast<unsigned>(t.boxFlags & 0xFFFFFFFFu), static_cast<unsigned>(t.boxFlags >> 32),
          t.statusV2 ? "v2" : "v1", ageMs(t.tStatus, now));
  s += "\"motors\":[";
  for (size_t i = 0; i < t.motors.size(); ++i) appendf(s, i ? ",%u" : "%u", t.motors[i]);
  appendf(s, "],\"motor_age\":%ld,", ageMs(t.tMotor, now));
  s += "\"rc\":[";
  for (size_t i = 0; i < t.rc.size(); ++i) appendf(s, i ? ",%u" : "%u", t.rc[i]);
  appendf(s, "],\"rc_age\":%ld,", ageMs(t.tRc, now));
  appendf(s, "\"imu\":{\"acc\":[%.3f,%.3f,%.3f],\"gyro\":[%.0f,%.0f,%.0f],\"mag\":[%d,%d,%d],\"age\":%ld},",
          t.acc[0], t.acc[1], t.acc[2], t.gyro[0], t.gyro[1], t.gyro[2], t.mag[0], t.mag[1], t.mag[2], ageMs(t.tImu, now));
  appendf(s, "\"alt\":{\"m\":%.2f,\"vs\":%.2f,\"baro\":%s,\"age\":%ld},", t.altM, t.varioMs,
          t.haveBaro ? std::to_string(t.baroAltM).c_str() : "null", ageMs(t.tAlt, now));
  appendf(s, "\"rates\":{\"att\":%.1f,\"bat\":%.1f,\"motor\":%.1f,\"rc\":%.1f,\"imu\":%.1f,\"alt\":%.1f,\"status\":%.1f,\"req\":%.1f,\"timeouts\":%.1f,\"bad\":%u},",
          r.att, r.bat, r.motor, r.rc, r.imu, r.alt, r.status, r.req, r.timeouts, bad);
  appendf(s, "\"api\":\"%d.%d\"}", t.apiMajor, t.apiMinor);
  return s;
}

std::string buildMeta(const Telemetry& t) {
  std::string s = "{\"type\":\"meta\",\"boxnames\":[";
  for (size_t i = 0; i < t.boxNames.size(); ++i) {
    if (i) s += ",";
    s += "\"" + jsonEscape(t.boxNames[i]) + "\"";
  }
  appendf(s, "],\"api\":\"%d.%d\",\"bat_src\":\"%s\",\"status_src\":\"%s\"}", t.apiMajor, t.apiMinor,
          t.noV2Analog ? "v1" : "v2", t.noV2Status ? "v1" : "v2");
  return s;
}

// ---- Serial -------------------------------------------------------------------------
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
  cfmakeraw(&tio);
  tio.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
  tio.c_cflag |= CS8 | CLOCAL | CREAD;
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;
  cfsetispeed(&tio, spd);
  cfsetospeed(&tio, spd);
  if (tcsetattr(fd, TCSANOW, &tio) != 0) { std::perror("tcsetattr"); ::close(fd); return -1; }
  tcflush(fd, TCIOFLUSH);
  return fd;
}

// ---- UDP (loopback only) -------------------------------------------------------------
int openUdp(const std::string& spec, sockaddr_in& dst) {
  const auto colon = spec.rfind(':');
  if (colon == std::string::npos) { std::fprintf(stderr, "--udp expects HOST:PORT\n"); return -2; }
  const std::string host = spec.substr(0, colon);
  const int port = std::atoi(spec.c_str() + colon + 1);
  std::memset(&dst, 0, sizeof dst);
  dst.sin_family = AF_INET;
  dst.sin_port = htons(static_cast<uint16_t>(port));
  if (port <= 0 || port > 65535 || inet_pton(AF_INET, host.c_str(), &dst.sin_addr) != 1) {
    std::fprintf(stderr, "bad --udp address '%s'\n", spec.c_str());
    return -2;
  }
  if ((ntohl(dst.sin_addr.s_addr) >> 24) != 127) {
    std::fprintf(stderr, "refusing --udp=%s: only loopback (127.x.x.x) is allowed\n", spec.c_str());
    return -2;
  }
  const int s = ::socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);
  if (s < 0) std::perror("socket");
  return s;
}

volatile std::sig_atomic_t g_stop = 0;
void onSignal(int) { g_stop = 1; }

}  // namespace

int main(int argc, char** argv) {
  std::vector<std::string> pos;
  std::string udpSpec = "127.0.0.1:14650";
  int maxReqHz = 120;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a.rfind("--udp=", 0) == 0) udpSpec = a.substr(6);
    else if (a.rfind("--max-req-hz=", 0) == 0) maxReqHz = std::clamp(std::atoi(a.c_str() + 13), 10, 300);
    else pos.push_back(a);
  }
  if (pos.empty()) {
    std::fprintf(stderr, "usage: %s <serial-port> [baud=115200] [print_hz=5] [--udp=127.0.0.1:14650|off] [--max-req-hz=120]\n", argv[0]);
    return 2;
  }
  const int baud = pos.size() > 1 ? std::atoi(pos[1].c_str()) : 115200;
  const int printHz = std::clamp(pos.size() > 2 ? std::atoi(pos[2].c_str()) : 5, 1, 20);

  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);

  sockaddr_in udpDst{};
  int udp = -1;
  if (udpSpec != "off") {
    udp = openUdp(udpSpec, udpDst);
    if (udp == -2) return 2;
  }

  const int fd = openPort(pos[0].c_str(), baud);
  if (fd < 0) return 1;
  std::fprintf(stderr, "drone_telemetry v2: %s, print %d Hz, max %d req/s, dashboard feed %s\n",
               pos[0].c_str(), printHz, maxReqHz, udp >= 0 ? udpSpec.c_str() : "off");

  const auto minGap = std::chrono::microseconds(1000000 / maxReqHz);
  const auto replyTimeout = std::chrono::milliseconds(50);
  const auto printPeriod = std::chrono::milliseconds(1000 / printHz);
  const auto udpPeriod = std::chrono::milliseconds(20);  // at most 50 datagrams/s

  Parser parser;
  Telemetry t;
  Rates rates;
  std::map<uint16_t, unsigned> replies;
  unsigned reqCount = 0, timeoutCount = 0;
  unsigned long seq = 0;
  size_t slot = 0;
  bool waiting = false;
  uint16_t waitingCmd = 0;
  std::vector<uint16_t> once = {MSP_API_VERSION, MSP_BOXNAMES};

  auto now = clk::now();
  clk::time_point sentAt = now, lastSend = now - minGap, lastReply = now;
  clk::time_point nextPrint = now, nextUdp = now, nextMeta = now + std::chrono::seconds(1);
  clk::time_point nextRates = now + std::chrono::seconds(1), nextBoxRetry = now + std::chrono::seconds(10);
  clk::time_point lastAttSent{};
  bool warned = false;

  auto effective = [&](uint16_t c) -> uint16_t {
    if (c == MSP2_INAV_ANALOG && t.noV2Analog) return MSP_ANALOG;
    if (c == MSP2_INAV_STATUS && t.noV2Status) return MSP_STATUS;
    return c;
  };

  while (!g_stop) {
    now = clk::now();

    // 1) send the next request when the previous one was answered (or timed out)
    if (waiting && now - sentAt > replyTimeout) { waiting = false; ++timeoutCount; }
    if (!waiting && now - lastSend >= minGap) {
      uint16_t cmd;
      if (!once.empty()) { cmd = once.front(); once.erase(once.begin()); }
      else { cmd = effective(kCycle[slot]); slot = (slot + 1) % kCycle.size(); }
      if (allowed(cmd)) {  // belt and braces: never send anything else
        const auto req = buildRequest(cmd);
        if (::write(fd, req.data(), req.size()) < 0) {
          if (errno != EAGAIN) { std::perror("write"); break; }
        } else {
          waiting = true; waitingCmd = cmd; sentAt = lastSend = now; ++reqCount;
        }
      }
    }

    // 2) read whatever the FC sent
    fd_set rf;
    FD_ZERO(&rf);
    FD_SET(fd, &rf);
    timeval tv{0, 2000};  // 2 ms
    if (::select(fd + 1, &rf, nullptr, nullptr, &tv) > 0) {
      uint8_t buf[512];
      const ssize_t n = ::read(fd, buf, sizeof buf);
      if (n < 0 && errno != EAGAIN && errno != EINTR) { std::perror("read"); break; }
      if (n == 0) { std::fprintf(stderr, "serial port closed\n"); break; }
      const auto rx = clk::now();
      for (ssize_t i = 0; i < n; ++i) {
        if (auto f = parser.feed(buf[i])) {
          apply(*f, t, rx);
          lastReply = rx;
          warned = false;
          if (!f->error) ++replies[f->cmd];
          if (waiting && f->cmd == waitingCmd) waiting = false;
        }
      }
    }
    now = clk::now();

    // 3) once a second: measured rates
    if (now >= nextRates) {
      const double dt = std::chrono::duration<double>(now - (nextRates - std::chrono::seconds(1))).count();
      auto hz = [&](std::initializer_list<uint16_t> ids) {
        unsigned c = 0;
        for (auto id : ids) c += replies[id];
        return c / dt;
      };
      rates.att = hz({MSP_ATTITUDE});
      rates.bat = hz({MSP2_INAV_ANALOG, MSP_ANALOG});
      rates.motor = hz({MSP_MOTOR});
      rates.rc = hz({MSP_RC});
      rates.imu = hz({MSP_RAW_IMU});
      rates.alt = hz({MSP_ALTITUDE});
      rates.status = hz({MSP2_INAV_STATUS, MSP_STATUS});
      rates.req = reqCount / dt;
      rates.timeouts = timeoutCount / dt;
      replies.clear();
      reqCount = timeoutCount = 0;
      nextRates = now + std::chrono::seconds(1);
    }

    // 4) dashboard feed: one datagram per new attitude sample, at most 50/s
    if (udp >= 0 && now >= nextUdp && t.tAtt != lastAttSent && t.any()) {
      const std::string js = buildJson(t, rates, ++seq, parser.badFrames, now);
      ::sendto(udp, js.data(), js.size(), 0, reinterpret_cast<const sockaddr*>(&udpDst), sizeof udpDst);
      lastAttSent = t.tAtt;
      nextUdp = now + udpPeriod;
    }
    if (udp >= 0 && now >= nextMeta) {
      const std::string js = buildMeta(t);
      ::sendto(udp, js.data(), js.size(), 0, reinterpret_cast<const sockaddr*>(&udpDst), sizeof udpDst);
      nextMeta = now + std::chrono::seconds(2);
    }
    if (t.boxNames.empty() && now >= nextBoxRetry) {
      once.push_back(MSP_BOXNAMES);
      nextBoxRetry = now + std::chrono::seconds(10);
    }

    // 5) human-readable journal line (same layout as v1, voltage now to 0.01 V)
    if (now >= nextPrint) {
      nextPrint = now + printPeriod;
      if (t.any()) {
        std::printf("roll %6.1f  pitch %6.1f  yaw %4d | %5.2f V  %5.2f A  %4u mAh  rssi %4u | loop %4u us  modes 0x%08x\n",
                    t.roll, t.pitch, t.yaw, t.vbat, t.amps, t.mah, t.rssi, t.cycleUs,
                    static_cast<unsigned>(t.boxFlags & 0xFFFFFFFFu));
        std::fflush(stdout);
      }
    }

    if (!warned && now - lastReply > std::chrono::seconds(3)) {
      std::fprintf(stderr, "no valid MSP reply for 3 s (wrong port? Configurator holding the USB port?)\n");
      warned = true;
    }
  }
  ::close(fd);
  if (udp >= 0) ::close(udp);
  return 0;
}
