#include <array>
#include <cstring>
#include <string>

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include "common/watchdog.h"
#include "common/util.h"
#include "system/hardware/hw.h"

const std::string watchdog_fn_prefix = Path::shm_path() + "/wd_";  // + <pid>
const std::string watchdog_phase_fn_prefix = Path::shm_path() + "/wd_phase_";  // + <pid>
constexpr size_t WATCHDOG_PHASE_SIZE = 64;

bool watchdog_kick(uint64_t ts) {
  static std::string fn = watchdog_fn_prefix + std::to_string(getpid());
  return util::write_file(fn.c_str(), &ts, sizeof(ts), O_WRONLY | O_CREAT) > 0;
}

bool watchdog_set_phase(const char *phase) {
  static char *phase_buffer = []() -> char * {
    const std::string fn = watchdog_phase_fn_prefix + std::to_string(getpid());
    const int fd = open(fn.c_str(), O_RDWR | O_CREAT | O_TRUNC | O_CLOEXEC, 0664);
    if (fd < 0 || ftruncate(fd, WATCHDOG_PHASE_SIZE) != 0) {
      if (fd >= 0) {
        close(fd);
      }
      return nullptr;
    }

    void *buffer = mmap(nullptr, WATCHDOG_PHASE_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);
    return buffer == MAP_FAILED ? nullptr : static_cast<char *>(buffer);
  }();

  if (phase_buffer == nullptr) {
    return false;
  }

  std::array<char, WATCHDOG_PHASE_SIZE> value = {};
  std::strncpy(value.data(), phase, value.size() - 1);
  std::memcpy(phase_buffer, value.data(), value.size());
  return true;
}
