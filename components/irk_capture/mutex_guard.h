#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

namespace esphome {
namespace irk_capture {

// Scope-based FreeRTOS mutex guard with optional timeout.
// Keep critical sections short: copy shared data out before logging, formatting,
// or calling NVS/BLE APIs. Holding a mutex across slow UART logging on
// single-core chips (e.g. ESP32-C3) can starve NimBLE and drop connections.
class MutexGuard {
 public:
  explicit MutexGuard(SemaphoreHandle_t mutex, TickType_t timeout = portMAX_DELAY)
      : mutex_(mutex), acquired_(false) {
    if (mutex_) {
      acquired_ = (xSemaphoreTake(mutex_, timeout) == pdTRUE);
    }
  }

  ~MutexGuard() {
    if (mutex_ && acquired_) {
      xSemaphoreGive(mutex_);
    }
  }

  // False if the wait timed out, or if there is no mutex. Always true for the
  // default, unbounded wait on a real mutex.
  bool acquired() const {
    return acquired_;
  }

  // Disable copy and move
  MutexGuard(const MutexGuard&) = delete;
  MutexGuard& operator=(const MutexGuard&) = delete;

 private:
  SemaphoreHandle_t mutex_;
  bool acquired_;
};

}  // namespace irk_capture
}  // namespace esphome
