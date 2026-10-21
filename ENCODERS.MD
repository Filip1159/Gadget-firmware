# Encoders and PCNT counting

## Result

`TICKS_PER_WHEEL_REV = 1920` (measured: one hand-turned wheel revolution read ~1920 on the PCNT counter).

The ESP32 PCNT counts **both edges of a single channel (A)**; channel B is only used to decide the direction (option *b*, x2 decoding). It does **not** count the edges of both channels (that would be x4 = 3840).

## Derivation

Motor: SJ01 1:120 gearbox, 6 V, 160 rpm, Hall encoder with 8 pulses per motor shaft revolution per channel (botland product 6287; figure taken from the old Arduino Uno R4 project, not re-verified here).

| Decoding | Counts per wheel rev | Formula |
|---|---|---|
| x1: rising edges of one channel | 960 | 8 * 120 |
| **x2: both edges of one channel** | **1920** | 8 * 120 * 2 |
| x4: both edges of both channels | 3840 | 8 * 120 * 4 |

The measured 1920 matches x2 only. The firmware configuration agrees (`SteeringManager::init`, per wheel):

- `pulse_gpio_num` = channel A, `ctrl_gpio_num` = channel B. Only one PCNT channel (`PCNT_CHANNEL_0`) is configured per wheel, so B is never counted as a pulse input.
- `pos_mode = PCNT_COUNT_INC` (rising edge of A: +1), `neg_mode = PCNT_COUNT_DEC` (falling edge of A: -1).
- `hctrl_mode = PCNT_MODE_REVERSE`, `lctrl_mode = PCNT_MODE_KEEP`: while B is high the count direction is inverted.

In quadrature, B is low at one kind of A edge and high at the other, depending on the direction of rotation. Turning one way: A rises with B low (+1), A falls with B high (inverted DEC = +1), so every edge counts +1. Turning the other way: A rises with B high (inverted INC = -1), A falls with B low (-1), so every edge counts -1. That is x2 decoding with a signed result, and it is why the old Uno R4 project saw 960 (it counted one edge type only).

Other notes:
- The ESP-IDF header (`driver/pcnt.h`) documents `pos_mode`/`neg_mode` as the counter mode on the positive/negative edge of the pulse pin, and `hctrl_mode`/`lctrl_mode` as the mode while the control pin is high/low.
- The glitch filter (`PCNT_FILTER_CYCLES = 100` APB cycles, about 1.25 us at 80 MHz) is far shorter than any real pulse and does not change the count.
- Signs (verified with `-DDEBUG_TICKS`): raw PCNT deltas are negative for FORWARD on both wheels and positive for BACKWARD, and LEFT/RIGHT spins give opposite signs, so `LEFT_ENCODER_SIGN` and `RIGHT_ENCODER_SIGN` are both -1..

## If this ever stops matching

If a re-measurement gives something other than 1920 (or left and right differ), record here: raw start/end counter values, how many revolutions, which wheel, direction, battery voltage, and whether the value changes with turning speed (which would point to missed edges or the glitch filter).
