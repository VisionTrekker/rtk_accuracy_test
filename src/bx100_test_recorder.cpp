#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <memory>
#include <limits>
#include <sstream>
#include <string>
#include <termios.h>
#include <vector>
#include <fcntl.h>
#include <unistd.h>

#include "geometry_msgs/msg/quaternion_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rtk_accuracy_test/msg/rtk_status.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/string.hpp"

namespace fs = std::filesystem;

namespace {

constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
constexpr int64_t kGpsEpochUnix = 315964800;

std::vector<std::string> split(const std::string &value, char delimiter)
{
  std::vector<std::string> result;
  std::stringstream stream(value);
  std::string item;
  while (std::getline(stream, item, delimiter)) result.push_back(item);
  if (!value.empty() && value.back() == delimiter) result.emplace_back();
  return result;
}

std::string without_checksum(const std::string &line)
{
  const auto star = line.find('*');
  return line.substr(0, star == std::string::npos ? line.size() : star);
}

std::vector<std::string> split_protocol_frames(const std::string &line)
{
  // Some receivers can start a new ASCII log before a long diagnostic log
  // emits its line ending. Preserve the physical line in the raw log, then
  // recover each protocol frame for parsing at '$' or '#'.
  std::vector<std::string> frames;
  size_t begin = 0;
  for (size_t index = 1; index < line.size(); ++index) {
    if (line[index] == '$' || line[index] == '#') {
      frames.push_back(line.substr(begin, index - begin));
      begin = index;
    }
  }
  frames.push_back(line.substr(begin));
  return frames;
}

bool nmea_checksum_valid(const std::string &line)
{
  if (line.empty() || line.front() != '$') return false;
  const auto star = line.find('*');
  if (star == std::string::npos) return true;
  if (star + 3 != line.size()) return false;
  unsigned int expected = 0;
  try { expected = std::stoul(line.substr(star + 1, 2), nullptr, 16); }
  catch (...) { return false; }
  unsigned char actual = 0;
  for (size_t i = 1; i < star; ++i) actual ^= static_cast<unsigned char>(line[i]);
  return actual == expected;
}

bool ascii_crc32_valid(const std::string &line)
{
  // BX100 ASCII logs use the NovAtel-compatible CRC-32: bytes after '#',
  // initial CRC 0, reversed polynomial 0xEDB88320, no final XOR.
  if (line.empty() || line.front() != '#') return false;
  const auto star = line.rfind('*');
  if (star == std::string::npos || star + 9 != line.size()) return false;
  unsigned long expected = 0;
  try {
    size_t consumed = 0;
    expected = std::stoul(line.substr(star + 1, 8), &consumed, 16);
    if (consumed != 8 || expected > 0xffffffffUL) return false;
  } catch (...) { return false; }
  uint32_t crc = 0;
  for (size_t index = 1; index < star; ++index) {
    crc ^= static_cast<uint8_t>(line[index]);
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 1U) ? ((crc >> 1U) ^ 0xEDB88320U) : (crc >> 1U);
    }
  }
  return crc == static_cast<uint32_t>(expected);
}

double number(const std::string &value, double fallback = kNan)
{
  if (value.empty()) return fallback;
  try {
    size_t used = 0;
    const double parsed = std::stod(value, &used);
    return used == value.size() ? parsed : fallback;
  } catch (...) { return fallback; }
}

int integer(const std::string &value, int fallback = -1)
{
  if (value.empty()) return fallback;
  try {
    size_t used = 0;
    const int parsed = std::stoi(value, &used);
    return used == value.size() ? parsed : fallback;
  } catch (...) { return fallback; }
}

double nmea_coordinate(const std::string &value, const std::string &direction)
{
  if (value.empty() || direction.size() != 1) return kNan;
  const char d = direction.front();
  const size_t degree_digits = (d == 'N' || d == 'S') ? 2 :
      (d == 'E' || d == 'W') ? 3 : 0;
  if (degree_digits == 0 || value.size() <= degree_digits) return kNan;
  const double degrees = number(value.substr(0, degree_digits));
  const double minutes = number(value.substr(degree_digits));
  if (!std::isfinite(degrees) || !std::isfinite(minutes)) return kNan;
  const double result = degrees + minutes / 60.0;
  return (d == 'S' || d == 'W') ? -result : result;
}

struct Gga {
  bool valid{false};
  int quality{-1};
  double latitude{kNan}, longitude{kNan}, altitude{kNan};
  double hdop{kNan}, differential_age{kNan};
  int station_id{-1};
  int satellites{-1};
  std::string utc;
};

struct Gst {
  bool valid{false};
  double std_lat{kNan}, std_lon{kNan}, std_alt{kNan};
};

struct BestPos {
  bool valid{false};
  std::string solution_status, position_type;
  double latitude{kNan}, longitude{kNan}, altitude{kNan};
  double std_lat{kNan}, std_lon{kNan}, std_alt{kNan};
};

struct Heading {
  bool valid{false};
  bool qualified{false};
  std::string source;
  std::string solution_status, position_type;
  double baseline{kNan}, heading{kNan}, pitch{kNan};
  double heading_std{kNan}, pitch_std{kNan};
};

bool parse_gga(const std::string &line, Gga &out)
{
  if (!nmea_checksum_valid(line)) return false;
  const auto fields = split(without_checksum(line), ',');
  if (fields.size() < 14 || fields[0].size() < 6 || fields[0].substr(3) != "GGA") return false;
  out.utc = fields[1];
  out.latitude = nmea_coordinate(fields[2], fields[3]);
  out.longitude = nmea_coordinate(fields[4], fields[5]);
  out.quality = integer(fields[6]);
  out.satellites = integer(fields[7]);
  out.hdop = number(fields[8]);
  out.altitude = number(fields[9]);
  out.differential_age = number(fields[13]);
  out.station_id = fields.size() > 14 ? integer(fields[14]) : -1;
  out.valid = out.quality > 0 && std::isfinite(out.latitude) && std::isfinite(out.longitude);
  return true;
}

bool parse_gst(const std::string &line, Gst &out)
{
  if (!nmea_checksum_valid(line)) return false;
  const auto fields = split(without_checksum(line), ',');
  if (fields.size() < 9 || fields[0].size() < 6 || fields[0].substr(3) != "GST") return false;
  out.std_lat = number(fields[6]);
  out.std_lon = number(fields[7]);
  out.std_alt = number(fields[8]);
  out.valid = std::isfinite(out.std_lat) && std::isfinite(out.std_lon) && std::isfinite(out.std_alt);
  return true;
}

bool parse_bestpos(const std::string &line, BestPos &out)
{
  if (line.rfind("#BESTPOSA", 0) != 0 && line.rfind("#BESTGNSSPOSA", 0) != 0) return false;
  if (!ascii_crc32_valid(line)) return false;
  const auto semi = line.find(';');
  if (semi == std::string::npos) return false;
  const auto fields = split(without_checksum(line.substr(semi + 1)), ',');
  if (fields.size() < 10) return false;
  out.solution_status = fields[0];
  out.position_type = fields[1];
  out.latitude = number(fields[2]);
  out.longitude = number(fields[3]);
  out.altitude = number(fields[4]);
  out.std_lat = number(fields[7]);
  out.std_lon = number(fields[8]);
  out.std_alt = number(fields[9]);
  out.valid = std::isfinite(out.latitude) && std::isfinite(out.longitude);
  return true;
}

bool parse_heading(const std::string &line, Heading &out)
{
  if (line.rfind("#UNIHEADINGA", 0) != 0) return false;
  if (!ascii_crc32_valid(line)) return false;
  const auto semi = line.find(';');
  if (semi == std::string::npos) return false;
  const auto fields = split(without_checksum(line.substr(semi + 1)), ',');
  if (fields.size() < 8) return false;
  out.solution_status = fields[0];
  out.position_type = fields[1];
  out.source = "UNIHEADINGA";
  out.baseline = number(fields[2]);
  out.heading = number(fields[3]);
  out.pitch = number(fields[4]);
  out.heading_std = number(fields[6]);
  out.pitch_std = number(fields[7]);
  out.valid = std::isfinite(out.heading);
  out.qualified = out.valid && out.solution_status == "SOL_COMPUTED";
  return true;
}

bool parse_hdt(const std::string &line, Heading &out)
{
  if (!nmea_checksum_valid(line)) return false;
  const auto fields = split(without_checksum(line), ',');
  if (fields.size() < 3 || fields[0].size() < 6 || fields[0].substr(3) != "HDT") return false;
  const double heading = number(fields[1]);
  if (!std::isfinite(heading) || heading < 0.0 || heading > 360.0 || fields[2] != "T") return false;
  out.valid = true;
  // NMEA HDT contains no solution quality or heading standard deviation.
  // Preserve it in /rtk/status, but do not call it a qualified RTK heading.
  out.qualified = false;
  out.source = "HDT";
  out.solution_status = "NMEA_HDT";
  out.position_type = "HEADING_UNQUALIFIED";
  // NMEA HDT is already clockwise from true north.
  out.heading = std::fmod(heading, 360.0);
  out.pitch = kNan;
  out.heading_std = kNan;
  out.pitch_std = kNan;
  out.baseline = kNan;
  return true;
}

std::string gga_position_type(int quality)
{
  switch (quality) {
    case 0: return "NO_FIX";
    case 1: return "SINGLE";
    case 2: return "DGPS";
    case 3: return "PPS";
    case 4: return "RTK_FIXED";
    case 5: return "RTK_FLOAT";
    case 6: return "ESTIMATED";
    case 7: return "MANUAL";
    case 8: return "SIMULATION";
    default: return "GGA_UNKNOWN";
  }
}

uint8_t gga_state_code(int quality)
{
  switch (quality) {
    case 4: return 1;  // RTK fixed according to NMEA GGA quality.
    case 5: return 2;  // RTK float.
    case 2: return 3;  // Differential GNSS.
    case 1:
    case 3:
    case 6:
    case 7:
    case 8: return 4;  // Non-RTK solution.
    default: return 0;
  }
}

uint8_t state_code(const std::string &position_type, const std::string &solution_status)
{
  if (position_type.find("FIXED") != std::string::npos || position_type.find("RTKFIXED") != std::string::npos) return 1;
  if (position_type.find("FLOAT") != std::string::npos || position_type.find("RTKFLOAT") != std::string::npos) return 2;
  if (position_type.find("DGPS") != std::string::npos || position_type.find("DGNSS") != std::string::npos) return 3;
  if (solution_status == "SOL_COMPUTED" || position_type.find("SINGLE") != std::string::npos || position_type.find("AUTONOMOUS") != std::string::npos) return 4;
  return 0;
}

class Serial {
public:
  ~Serial() { close(); }
  bool open(const std::string &path, int baud, std::string &error)
  {
    fd_ = ::open(path.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) { error = std::strerror(errno); return false; }
    termios tty{};
    if (tcgetattr(fd_, &tty) != 0) { error = std::strerror(errno); close(); return false; }
    speed_t speed = baud == 115200 ? B115200 : baud == 230400 ? B230400 : baud == 57600 ? B57600 : 0;
    if (speed == 0) { error = "unsupported baud rate"; close(); return false; }
    cfmakeraw(&tty); cfsetispeed(&tty, speed); cfsetospeed(&tty, speed);
    tty.c_cflag |= CLOCAL | CREAD; tty.c_cflag &= ~CRTSCTS;
    if (tcsetattr(fd_, TCSANOW, &tty) != 0) { error = std::strerror(errno); close(); return false; }
    path_ = path; return true;
  }
  void close() { if (fd_ >= 0) ::close(fd_); fd_ = -1; path_.clear(); }
  bool is_open() const { return fd_ >= 0; }
  std::ptrdiff_t read(char *buffer, size_t size, std::string &error)
  {
    if (fd_ < 0) { error = "serial port is not open"; return -1; }
    const auto n = ::read(fd_, buffer, size);
    if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) { error = std::strerror(errno); return -1; }
    return n < 0 ? 0 : n;
  }
private:
  int fd_{-1}; std::string path_;
};

}  // namespace

class TestRecorder final : public rclcpp::Node {
public:
  TestRecorder() : Node("rtk_test_recorder")
  {
    port_ = declare_parameter<std::string>("port", "/dev/ttyACM0");
    baud_ = declare_parameter<int>("baud_rate", 115200);
    frame_id_ = declare_parameter<std::string>("frame_id", "rtk");
    log_directory_ = declare_parameter<std::string>("log_directory", "./data");
    receiver_role_ = declare_parameter<std::string>("receiver_role", "rover");
    heading_offset_ = declare_parameter<double>("heading_offset_deg", 0.0);
    publish_heading_ = declare_parameter<bool>("publish_heading", true);
    publish_unqualified_hdt_ = declare_parameter<bool>("publish_unqualified_hdt", false);
    fs::create_directories(log_directory_);
    const auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::tm tm{}; localtime_r(&now, &tm);
    std::ostringstream stamp; stamp << std::put_time(&tm, "%Y%m%d_%H%M%S");
    raw_.open((fs::path(log_directory_) / (receiver_role_ + "_" + stamp.str() + ".raw")).string());
    if (!serial_.open(port_, baud_, error_)) RCLCPP_ERROR(get_logger(), "serial: %s", error_.c_str());
    fix_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("fix", 20);
    status_pub_ = create_publisher<rtk_accuracy_test::msg::RtkStatus>("rtk/status", 20);
    raw_pub_ = create_publisher<std_msgs::msg::String>("rtk/raw", 100);
    heading_pub_ = create_publisher<geometry_msgs::msg::QuaternionStamped>("heading", 20);
    timer_ = create_wall_timer(std::chrono::milliseconds(10), std::bind(&TestRecorder::poll, this));
    RCLCPP_INFO(get_logger(), "recording %s", raw_.is_open() ? raw_.tellp() >= 0 ? "raw serial data" : "" : "without raw file");
  }
  ~TestRecorder() override { serial_.close(); if (raw_.is_open()) raw_.close(); }

private:
  void poll()
  {
    if (!serial_.is_open()) return;
    char buffer[4096]; std::string error;
    const auto n = serial_.read(buffer, sizeof(buffer), error);
    if (n < 0) { RCLCPP_ERROR_THROTTLE(get_logger(), *this, 2000, "serial read: %s", error.c_str()); return; }
    if (n == 0) return;
    rx_.append(buffer, static_cast<size_t>(n));
    size_t newline = 0;
    while ((newline = rx_.find('\n')) != std::string::npos) {
      std::string line = rx_.substr(0, newline); rx_.erase(0, newline + 1);
      if (!line.empty() && line.back() == '\r') line.pop_back();
      if (line.empty()) continue;
      ++lines_received_; const auto receipt = now();
      const auto epoch_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
      if (raw_.is_open()) raw_ << epoch_ns << '\t' << line << '\n' << std::flush;
      std_msgs::msg::String raw_msg; raw_msg.data = line; raw_pub_->publish(raw_msg);
      for (const auto &frame : split_protocol_frames(line)) {
        if (!frame.empty()) process(frame, receipt);
      }
    }
    if (rx_.size() > 65536) { rx_.clear(); ++parse_errors_; }
  }

  void process(const std::string &line, const rclcpp::Time &receipt)
  {
    bool parsed = false;
    Gga gga; Gst gst; BestPos best; Heading heading;
    if (line.size() > 6 && line[0] == '$' && line.substr(3, 3) == "GGA") {
      if (parse_gga(line, gga)) { last_gga_ = gga; parsed = true; publish_fix(receipt); }
      else { ++checksum_errors_; }
    } else if (line.size() > 6 && line[0] == '$' && line.substr(3, 3) == "GST") {
      if (parse_gst(line, gst)) { last_gst_ = gst; parsed = true; }
      else ++checksum_errors_;
    } else if (line.rfind("#BESTPOSA", 0) == 0 || line.rfind("#BESTGNSSPOSA", 0) == 0) {
      if (parse_bestpos(line, best)) { last_best_ = best; parsed = true; }
      else ++parse_errors_;
    } else if (line.rfind("#UNIHEADINGA", 0) == 0) {
      if (parse_heading(line, heading)) {
        last_heading_ = heading;
        parsed = true;
        if (publish_heading_ && last_heading_.qualified) publish_heading(receipt);
      }
      else ++parse_errors_;
    } else if (line.size() > 6 && line[0] == '$' && line.substr(3, 3) == "HDT") {
      if (parse_hdt(line, heading)) {
        last_heading_ = heading;
        parsed = true;
        if (publish_heading_ && publish_unqualified_hdt_) publish_heading(receipt);
      }
      else ++checksum_errors_;
    } else if (line.front() == '#' && !ascii_crc32_valid(line)) {
      ++checksum_errors_;
    }
    if (parsed) publish_status(receipt);
  }

  void publish_fix(const rclcpp::Time &stamp)
  {
    sensor_msgs::msg::NavSatFix fix; fix.header.stamp = stamp; fix.header.frame_id = frame_id_;
    fix.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
    fix.status.status = last_gga_.valid ? sensor_msgs::msg::NavSatStatus::STATUS_FIX : sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
    fix.latitude = last_gga_.latitude; fix.longitude = last_gga_.longitude; fix.altitude = last_gga_.altitude;
    if (last_gst_.valid) {
      fix.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
      fix.position_covariance[0] = last_gst_.std_lon * last_gst_.std_lon;
      fix.position_covariance[4] = last_gst_.std_lat * last_gst_.std_lat;
      fix.position_covariance[8] = last_gst_.std_alt * last_gst_.std_alt;
    } else fix.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
    fix_pub_->publish(fix);
  }

  void publish_heading(const rclcpp::Time &stamp)
  {
    // /heading is a standard ROS ENU quaternion: yaw is counter-clockwise
    // from east. RtkStatus.heading remains clockwise from true north.
    double clockwise_from_north = last_heading_.heading;
    if (last_heading_.source == "UNIHEADINGA") {
      // BX100 UNIHEADINGA uses counter-clockwise angle from true north.
      clockwise_from_north = 360.0 - clockwise_from_north;
    }
    clockwise_from_north = std::fmod(clockwise_from_north + heading_offset_ + 360.0, 360.0);
    const double yaw = (90.0 - clockwise_from_north) * M_PI / 180.0;
    geometry_msgs::msg::QuaternionStamped message; message.header.stamp = stamp; message.header.frame_id = frame_id_;
    message.quaternion.z = std::sin(yaw / 2.0); message.quaternion.w = std::cos(yaw / 2.0);
    heading_pub_->publish(message);
  }

  void publish_status(const rclcpp::Time &stamp)
  {
    rtk_accuracy_test::msg::RtkStatus message; message.header.stamp = stamp; message.header.frame_id = frame_id_;
    message.valid = last_gga_.valid || last_best_.valid;
    message.utc_time = last_gga_.utc;
    message.solution_status = last_best_.valid ? last_best_.solution_status : "GGA_ONLY";
    message.position_type = last_best_.valid ? last_best_.position_type : gga_position_type(last_gga_.quality);
    message.state_code = last_best_.valid ?
      state_code(last_best_.position_type, last_best_.solution_status) : gga_state_code(last_gga_.quality);
    message.latitude = last_gga_.latitude; message.longitude = last_gga_.longitude; message.altitude = last_gga_.altitude;
    if (last_best_.valid) { message.latitude = last_best_.latitude; message.longitude = last_best_.longitude; message.altitude = last_best_.altitude; }
    message.latitude_std = last_best_.std_lat; message.longitude_std = last_best_.std_lon; message.altitude_std = last_best_.std_alt;
    message.gst_std_latitude = last_gst_.std_lat; message.gst_std_longitude = last_gst_.std_lon; message.gst_std_altitude = last_gst_.std_alt;
    message.differential_age = last_gga_.differential_age; message.differential_station_id = last_gga_.station_id;
    message.satellites_used = std::max(0, last_gga_.satellites); message.hdop = last_gga_.hdop;
    message.heading_valid = last_heading_.valid && last_heading_.qualified;
    message.heading_source = last_heading_.source;
    double heading_cw_from_true_north = last_heading_.heading;
    if (last_heading_.source == "UNIHEADINGA") {
      heading_cw_from_true_north = std::fmod(360.0 - heading_cw_from_true_north, 360.0);
    }
    message.heading = heading_cw_from_true_north;
    message.pitch = last_heading_.pitch; message.heading_std = last_heading_.heading_std; message.pitch_std = last_heading_.pitch_std; message.baseline_length = last_heading_.baseline;
    message.lines_received = lines_received_; message.checksum_errors = checksum_errors_; message.parse_errors = parse_errors_; message.interline_gap_errors = 0;
    status_pub_->publish(message);
  }

  std::string port_, frame_id_, log_directory_, receiver_role_, error_, rx_;
  int baud_{115200}; double heading_offset_{0.0};
  bool publish_heading_{true}, publish_unqualified_hdt_{false};
  Serial serial_; std::ofstream raw_; rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;
  rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr heading_pub_;
  rclcpp::Publisher<rtk_accuracy_test::msg::RtkStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr raw_pub_;
  Gga last_gga_; Gst last_gst_; BestPos last_best_; Heading last_heading_;
  uint32_t lines_received_{0}, checksum_errors_{0}, parse_errors_{0};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<TestRecorder>()); rclcpp::shutdown(); return 0;
}
