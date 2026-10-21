#pragma once

#include <stdint.h>

// Turns raw readings of a free-running int16 hardware counter (ESP32 PCNT) into signed tick deltas.
// Hardware-free, host-tested. The counter is diffed against the previous reading and only cleared when
// it nears its limit: the hardware counter resets to 0 on reaching +-32767, which would corrupt a delta,
// so it must never get there (a control tick moves it by a few hundred at most, far below the
// 16000 threshold's margin). The few pulses that can arrive between the read and the clear are the
// only ones lost, about one clear per 8 wheel revolutions.
class EncoderTracker {
   public:
    struct Reading {
        int16_t delta;     // ticks since the previous update (signed)
        bool clearCounter;  // caller must clear the hardware counter now
    };

    static constexpr int16_t CLEAR_THRESHOLD = 16000;

    Reading update(int16_t raw) {
        const int16_t delta = static_cast<int16_t>(raw - lastRaw);
        lastRaw = raw;
        const bool clear = raw > CLEAR_THRESHOLD || raw < -CLEAR_THRESHOLD;
        if (clear) {
            lastRaw = 0;
        }
        return {delta, clear};
    }

   private:
    int16_t lastRaw = 0;
};
