#ifndef PYLON_DEBUG_H
#define PYLON_DEBUG_H

#include <stdint.h>
#include <mutex>
#include "../devboard/utils/types.h"

/* Bench tool for finding out how a BMS reacts to the Pylon commands. Frames are queued from the web
   route /pylondbg or the USB serial console (devboard/debug/pylon_debug.cpp) and sent by the Pylon
   driver from the core loop, so the CAN controller is only ever driven from one task. Nothing here is
   stored: a reboot returns to the saved settings with nothing muted, queued or repeating. */
class PylonDebug {
 public:
  static const uint8_t MUTE_NONE = 0;   // Driver sends everything as usual
  static const uint8_t MUTE_RELAY = 1;  // Driver skips its 0x820x/0x821x/0x822x frames, keeps polling
  static const uint8_t MUTE_ALL = 2;    // Driver sends nothing, only frames queued here go out
  static const int QUEUE_SIZE = 16;
  static const int PERIODIC_SLOTS = 8;

  struct Periodic {
    bool active = false;
    uint32_t period_ms = 0;
    unsigned long last_ms = 0;
    CAN_frame frame = {};
  };

  volatile uint8_t mute = MUTE_NONE;

  bool enqueue(const CAN_frame& frame) {
    std::lock_guard<std::mutex> lock(mutex);
    if (count >= QUEUE_SIZE) {
      return false;
    }
    queue[(head + count) % QUEUE_SIZE] = frame;
    count++;
    return true;
  }

  // frame == nullptr switches the slot off
  bool set_periodic(int slot, uint32_t period_ms, const CAN_frame* frame) {
    if (slot < 0 || slot >= PERIODIC_SLOTS) {
      return false;
    }
    std::lock_guard<std::mutex> lock(mutex);
    periodic[slot].active = frame != nullptr;
    if (frame != nullptr) {
      periodic[slot].frame = *frame;
      periodic[slot].period_ms = period_ms;
      periodic[slot].last_ms = 0;
      periodic[slot].frame_due_now = true;
    }
    return true;
  }

  void clear_periodic() {
    std::lock_guard<std::mutex> lock(mutex);
    for (auto& p : periodic) {
      p.active = false;
    }
  }

  Periodic get_periodic(int slot) {
    std::lock_guard<std::mutex> lock(mutex);
    return periodic[slot];
  }

  // Called from the core loop. Sends at most a few frames per call so the loop never stalls.
  template <typename SendFn>
  void service(unsigned long now, SendFn send) {
    CAN_frame out[QUEUE_SIZE + PERIODIC_SLOTS];
    int n = 0;
    {
      std::lock_guard<std::mutex> lock(mutex);
      while (count > 0) {
        out[n++] = queue[head];
        head = (head + 1) % QUEUE_SIZE;
        count--;
      }
      for (auto& p : periodic) {
        if (p.active && (p.frame_due_now || now - p.last_ms >= p.period_ms)) {
          p.frame_due_now = false;
          p.last_ms = now;
          out[n++] = p.frame;
        }
      }
    }
    for (int i = 0; i < n; i++) {
      send(&out[i]);
    }
  }

 private:
  struct PeriodicSlot : Periodic {
    bool frame_due_now = false;
  };
  std::mutex mutex;
  CAN_frame queue[QUEUE_SIZE] = {};
  int head = 0;
  int count = 0;
  PeriodicSlot periodic[PERIODIC_SLOTS];
};

extern PylonDebug pylon_debug;

#endif
