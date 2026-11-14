#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

namespace esphome {
namespace irk_capture {

//======================== Thread Safety: RAII Mutex Guard ========================
/*
RAII wrapper for FreeRTOS mutex to ensure exception-safe locking.
Used to protect shared state accessed by both NimBLE task and ESPHome main task.

CRITICAL PERFORMANCE OPTIMIZATION:
Always minimize mutex hold time by copying data out before releasing the lock,
then performing slow operations (logging, NVS access) AFTER the lock is
released.

BAD (blocks NimBLE task during slow UART logging):
  {
    MutexGuard lock(state_mutex_);
    peer_id = post_disc_timers_[0].peer_id;
    ESP_LOGD(TAG, "Peer: %s", addr_to_str(peer_id).c_str()); // SLOW - UART
bottleneck!
  }

GOOD (minimal lock hold time):
  ble_addr_t peer_id_copy;
  {
    MutexGuard lock(state_mutex_);
    peer_id_copy = post_disc_timers_[0].peer_id;  // Fast memory copy
  }  // Lock released immediately
  ESP_LOGD(TAG, "Peer: %s", addr_to_str(peer_id_copy).c_str());  // Safe - no
lock held

Why: On ESP32-C3 (single core), holding a mutex during logging can cause the
NimBLE task to miss critical timing windows (e.g., supervision timeout), leading
to dropped connections.

Usage pattern:
  {
    MutexGuard lock(state_mutex_);
    // ONLY fast memory operations here (reads, writes, simple arithmetic)
  }  // Mutex automatically released
  // Logging, NVS access, string formatting happen here (outside critical
section)

A timeout may be given for a lock that must not stall its caller:
  MutexGuard lock(mutex, pdMS_TO_TICKS(200));
  if (!lock.acquired()) return;  // Try again later
*/
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
