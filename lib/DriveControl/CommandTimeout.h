#pragma once

#include <stdint.h>

// Failsafe: the frontend repeats the active direction every 500ms while a button/key is held.
// If nothing arrives within this window, the control task stops the robot by itself.
constexpr uint32_t COMMAND_TIMEOUT_MS = 1000;

// Hardware-free timeout rule. Times are millis() values; unsigned subtraction keeps it correct
// across the 32-bit wraparound.
class CommandTimeout {
   public:
    void refresh(uint32_t nowMs) {
        lastCommandMs = nowMs;
    }
    // `driving` is false while the robot is stopped, which can never time out.
    bool expired(uint32_t nowMs, bool driving) const {
        return driving && static_cast<uint32_t>(nowMs - lastCommandMs) >= COMMAND_TIMEOUT_MS;
    }

   private:
    uint32_t lastCommandMs = 0;
};
