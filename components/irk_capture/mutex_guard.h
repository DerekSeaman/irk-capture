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
*/
class MutexGuard {
 public:
  explicit MutexGuard(SemaphoreHandle_t mutex) : mutex_(mutex) {
    if (mutex_) {
      xSemaphoreTake(mutex_, portMAX_DELAY);
    }
  }

  ~MutexGuard() {
    if (mutex_) {
      xSemaphoreGive(mutex_);
    }
  }

  // Disable copy and move
  MutexGuard(const MutexGuard&) = delete;
  MutexGuard& operator=(const MutexGuard&) = delete;

 private:
  SemaphoreHandle_t mutex_;
};

}  // namespace irk_capture
}  // namespace esphome
