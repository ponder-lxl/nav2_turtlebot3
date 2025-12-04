// ROS2-aware logging helper: preserves optional stdout/stderr redirection to a file
#pragma once

#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <ctime>
#include <cstdio>
#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>
#include <cstring>
#include <sys/time.h>

#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging.h>

#include <cstdint>
#include <cstdarg>
#include <atomic>
#include <memory>

namespace loong_log {

// Default log directory (can be overridden by passing a path to initLogging)
static const std::string DEFAULT_LOG_DIR = "/home/rx01109/loong_nav/logs";

// Severity levels used by this helper
enum LogLevel { INFO = 0, WARN = 1, ERROR = 2 };

// ========== helper to format timestamps ==========
inline std::string currentTimestamp() {
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  std::time_t now_time = tv.tv_sec;
  struct tm local_tm;
  localtime_r(&now_time, &local_tm);
  char buf[64];
  std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &local_tm);
  std::ostringstream oss;
  oss << buf << "." << std::setw(3) << std::setfill('0') << (tv.tv_usec / 1000);
  return oss.str();
}

// Forward declare installation function so initLogging can call it even if
// the definition appears later in this header.
// control whether logs are echoed to the user interface (original stdout/stderr)
inline void installRos2OutputHandler();
inline void setConsoleOutputEnabled(bool enabled);
inline bool getConsoleOutputEnabled();

// internal globals
inline std::atomic<bool> g_console_output_enabled{true};
inline std::shared_ptr<std::ofstream> g_ofs_ptr{nullptr};
inline int g_orig_stdout_fd = -1;
inline int g_orig_stderr_fd = -1;
inline int g_log_fd = -1;


// Simple stream-like logger that emits to ROS2 logging when rclcpp is initialized,
// and always writes a prefixed line to stdout (which can be redirected to a file).
class LogStream {
public:
  LogStream(const char* file, int line, LogLevel level = INFO)
      : level_(level)
  {
    const char* filename = file;
    const char* p = strrchr(file, '/');
    if (p) filename = p + 1;
    std::ostringstream oss;
    oss << "[" << currentTimestamp() << "] "
        << "[" << filename << ":" << line << "] ";
    prefix_ = oss.str();
  }

  template<typename T>
  LogStream& operator<<(const T& value) {
    buffer_ << value;
    return *this;
  }

  ~LogStream() {
  // Message
  std::string msg = buffer_.str();

  // Only write our custom formatted line to stdout/stderr. Do NOT call rclcpp
  // logging here to avoid duplicate lines when ROS2 logging is active.
  const std::string line = prefix_ + msg + "\n";
  if (g_log_fd >= 0) {
    ssize_t ww = write(g_log_fd, line.c_str(), line.size());
    (void)ww;
  } else {
    std::ostream &out = (level_ == ERROR) ? std::cerr : std::cout;
    out << prefix_ << msg << std::endl;
  }

  // Optionally echo to original console (before we redirected fds)
  if (g_console_output_enabled.load()) {
    int fd = (level_ == ERROR) ? g_orig_stderr_fd : g_orig_stdout_fd;
    if (fd >= 0) {
      ssize_t w = write(fd, line.c_str(), line.size());
      (void)w;
    }
  }
  }

private:
  std::ostringstream buffer_;
  std::string prefix_;
  LogLevel level_;
};

// Convenience macros
#define LOONGLOG_INFO loong_log::LogStream(__FILE__, __LINE__, loong_log::INFO)
#define LOONGLOG_WARN loong_log::LogStream(__FILE__, __LINE__, loong_log::WARN)
#define LOONGLOG_ERROR loong_log::LogStream(__FILE__, __LINE__, loong_log::ERROR)

// Initialize optional file redirection. If `log_path_in` is empty the function will
// pick a sensible default. Returns true on success. This function does not touch
// rclcpp initialization; call rclcpp::init(...) elsewhere as needed.
inline bool initLogging(const std::string &log_path_in = "") {
  std::string path = log_path_in;
  if (path.empty()) {
    const char *env = getenv("LOG_FILE");
    if (env && env[0]) path = env;
  }

  if (path.empty()) {
    std::string root = DEFAULT_LOG_DIR;
    if (root.empty()) {
      const char *env_root = getenv("PLANNER_ROOT");
      if (env_root && env_root[0]) root = env_root;
    }
    if (root.empty()) {
      const char *home = getenv("HOME");
      if (home && home[0]) root = std::string(home) + "/planner_logs";
      else root = std::string("/tmp/planner_logs");
    }
    static std::string cached_time;
    if (cached_time.empty()) {
      std::time_t t = std::time(nullptr);
      char buf[64];
      std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", std::localtime(&t));
      cached_time = buf;
    }
    path = root + "/planner_" + cached_time + ".log";
  }

  // ensure parent directory exists (mkdir -p)
  auto ensure_parent_dir = [](const std::string &filepath) -> bool {
    auto pos = filepath.find_last_of('/');
    if (pos == std::string::npos) return true;
    std::string dir = filepath.substr(0, pos);
    if (dir.empty()) return true;
    std::string prefix;
    size_t start = 0;
    if (dir.size() > 0 && dir[0] == '/') { prefix = "/"; start = 1; }
    for (size_t i = start; i < dir.size(); ++i) {
      prefix.push_back(dir[i]);
      if (dir[i] == '/' || i + 1 == dir.size()) {
        std::string tocreate = prefix;
        if (tocreate.size() > 1 && tocreate.back() == '/') tocreate.pop_back();
        if (tocreate.empty()) continue;
        struct stat st;
        if (stat(tocreate.c_str(), &st) != 0) {
          if (mkdir(tocreate.c_str(), 0755) != 0) {
            if (errno != EEXIST) return false;
          }
        } else {
          if (!S_ISDIR(st.st_mode)) return false;
        }
      }
    }
    return true;
  };

  if (!ensure_parent_dir(path)) return false;

  int fd = open(path.c_str(), O_WRONLY | O_CREAT | O_APPEND, 0644);
  if (fd < 0) {
    fprintf(stderr, "initLogging: open(%s) failed errno=%d\n", path.c_str(), errno);
    return false;
  }

  const char *hdr = "=== planner log start ===\n";
  ssize_t w = write(fd, hdr, strlen(hdr));
  (void)w;

  // save original stdout/stderr fds so we can optionally echo to console
  g_orig_stdout_fd = dup(STDOUT_FILENO);
  g_orig_stderr_fd = dup(STDERR_FILENO);

    // keep the opened log fd and a file stream wrapper; do NOT dup2 into
    // STDOUT/STDERR to avoid indirect duplicate writes via both fd and rdbuf.
    g_log_fd = fd;

  // keep an ofstream handle so destructor/flush can be controlled if needed,
  // but do NOT rebind std::cout/std::cerr rdbuf to avoid creating a second
  // file descriptor that writes to the same path (which causes duplicates).
  g_ofs_ptr = std::make_shared<std::ofstream>(path, std::ios::app);

  // Attempt to install ROS2 output handler so ROS2 logs use our timestamp/file:line format.
  // This call is safe even if rclcpp hasn't been initialized; if needed, users can also
  // call loong_log::installRos2OutputHandler() explicitly after rclcpp::init().
  try {
    installRos2OutputHandler();
    // write a small notice to stderr before it's redirected (if already redirected, it's fine)
    fprintf(stderr, "loong_log: installed ROS2 output handler\n");
  } catch(...) {
    // ignore any failure installing the handler
  }

  return true;
}

inline void setConsoleOutputEnabled(bool enabled) {
  g_console_output_enabled.store(enabled);
}

inline bool getConsoleOutputEnabled() {
  return g_console_output_enabled.load();
}

} // namespace loong_log

// ============================================================================
// ROS2 logging output handler customization
// ============================================================================
namespace loong_log {

// rcutils output handler that formats ROS2 logs with system time and file:line
inline void ros2_output_handler(
  const rcutils_log_location_t * location,
  int severity,
  const char * name,
  rcutils_time_point_value_t timestamp,
  const char * format,
  va_list * args)
{
  (void)timestamp; // we'll use system time instead

  // severity string
  const char *sev = "INFO";
  if (severity == RCUTILS_LOG_SEVERITY_WARN) sev = "WARN";
  else if (severity == RCUTILS_LOG_SEVERITY_ERROR) sev = "ERROR";
  else if (severity == RCUTILS_LOG_SEVERITY_FATAL) sev = "FATAL";

  // format the message from format + va_list
  std::string msg;
  if (format) {
    // use a reasonably large fixed buffer
    char buf[8192];
    if (args) {
      va_list args_copy;
      va_copy(args_copy, *args);
      vsnprintf(buf, sizeof(buf), format, args_copy);
      va_end(args_copy);
    } else {
      // no va_list provided; treat format as a plain C-string format with no args
      snprintf(buf, sizeof(buf), "%s", format);
    }
    msg = buf;
  }

  // system time stamp
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  std::time_t now_time = tv.tv_sec;
  struct tm local_tm;
  localtime_r(&now_time, &local_tm);
  char timebuf[64];
  std::strftime(timebuf, sizeof(timebuf), "%Y-%m-%d %H:%M:%S", &local_tm);
  char msecbuf[16];
  std::snprintf(msecbuf, sizeof(msecbuf), ".%03u", static_cast<unsigned int>(tv.tv_usec / 1000));

  // file:line extraction
  const char *file = location && location->file_name ? location->file_name : "";
  const char *p = strrchr(file, '/');
  if (p) file = p + 1;
  int line = location ? location->line_number : 0;

  // final formatted output: [SEV] [TIME] [name]: [file:line] message
  std::ostringstream oss;
  oss << "[" << sev << "] "
      << "[" << timebuf << msecbuf << "] "
      << "[" << name << "]: "
      << "[" << file << ":" << line << "] "
      << msg << std::endl;

  const std::string outstr = oss.str();
  if (g_log_fd >= 0) {
    ssize_t ww = write(g_log_fd, outstr.c_str(), outstr.size());
    (void)ww;
  } else {
    fputs(outstr.c_str(), stderr);
  }

  if (g_console_output_enabled.load()) {
    int fd = STDOUT_FILENO;
    if (severity >= RCUTILS_LOG_SEVERITY_WARN) fd = g_orig_stderr_fd >= 0 ? g_orig_stderr_fd : STDERR_FILENO;
    else fd = g_orig_stdout_fd >= 0 ? g_orig_stdout_fd : STDOUT_FILENO;
    if (fd >= 0) {
      ssize_t ww = write(fd, outstr.c_str(), outstr.size());
      (void)ww;
    }
  }
}

// Install the custom ROS2 output handler. Call this after rclcpp::init().
inline void installRos2OutputHandler()
{
  // rcutils requires a function with this signature; pass our wrapper.
  rcutils_logging_set_output_handler(ros2_output_handler);
}

} // namespace loong_log

