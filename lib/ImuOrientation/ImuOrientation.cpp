#include "ImuOrientation.h"

#include <math.h>

namespace {

// Dominant axis of a 3-vector (index and sign), and the angle in degrees between the vector and that axis.
SignedAxis dominantAxis(const float v[3], float& offAxisDeg) {
    int best = 0;
    for (int i = 1; i < 3; i++) {
        if (fabsf(v[i]) > fabsf(v[best])) {
            best = i;
        }
    }
    const float magnitude = sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    const float cosine = magnitude > 0.0f ? fabsf(v[best]) / magnitude : 0.0f;
    offAxisDeg = acosf(fminf(cosine, 1.0f)) * 180.0f / 3.14159265f;
    SignedAxis axis;
    axis.axis = static_cast<int8_t>(best);
    axis.sign = v[best] < 0.0f ? -1 : 1;
    return axis;
}

float magnitude(const float v[3]) {
    return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

}  // namespace

const char* SignedAxis::name() const {
    static const char* const names[2][3] = {{"-X", "-Y", "-Z"}, {"+X", "+Y", "+Z"}};
    if (!valid() || axis > 2 || sign == 0) {
        return "?";
    }
    return names[sign > 0 ? 1 : 0][axis];
}

void ImuOrientationCheck::Mean::add(const ImuSample& s) {
    const float values[6] = {s.ax, s.ay, s.az, s.gx, s.gy, s.gz};
    for (int i = 0; i < 6; i++) {
        sum[i] += values[i];
    }
    count++;
}

void ImuOrientationCheck::start(uint32_t nowMs) {
    still = Mean();
    spin = Mean();
    outcome = Result();
    phase = Phase::Still;
    phaseStartMs = nowMs;
}

void ImuOrientationCheck::abort() {
    if (phase == Phase::Idle) {
        return;
    }
    phase = Phase::Idle;
    outcome = Result();
    outcome.verdict = Verdict::Aborted;
}

bool ImuOrientationCheck::update(uint32_t nowMs, const ImuSample* sample) {
    if (phase == Phase::Idle) {
        return false;
    }
    const uint32_t elapsed = nowMs - phaseStartMs;  // wraparound safe
    if (phase == Phase::Still) {
        if (sample) {
            still.add(*sample);
        }
        if (elapsed >= STILL_MS) {
            phase = Phase::Spin;
            phaseStartMs = nowMs;
        }
        return false;
    }
    if (sample && elapsed >= SPIN_SETTLE_MS) {
        spin.add(*sample);
    }
    if (elapsed < SPIN_MS) {
        return false;
    }
    phase = Phase::Idle;
    evaluate();
    return true;
}

void ImuOrientationCheck::evaluate() {
    if (still.count < MIN_SAMPLES || spin.count < MIN_SAMPLES) {
        outcome.verdict = Verdict::NoData;
        return;
    }

    const float accel[3] = {still.get(0), still.get(1), still.get(2)};
    const float bias[3] = {still.get(3), still.get(4), still.get(5)};
    if (magnitude(bias) > STILL_MAX_RATE_DPS) {
        outcome.verdict = Verdict::Moving;
        return;
    }
    const float gravity = magnitude(accel);
    if (gravity < GRAVITY_MIN_G || gravity > GRAVITY_MAX_G) {
        outcome.verdict = Verdict::NoGravity;
        return;
    }
    outcome.up = dominantAxis(accel, outcome.tiltDeg);

    // The spin rate with the stationary gyro offset removed.
    const float rate[3] = {spin.get(3) - bias[0], spin.get(4) - bias[1], spin.get(5) - bias[2]};
    if (magnitude(rate) < MIN_SPIN_RATE_DPS) {
        outcome.verdict = Verdict::NoSpin;
        return;
    }
    float spinOffAxisDeg;
    outcome.yaw = dominantAxis(rate, spinOffAxisDeg);
    outcome.rateDps = fabsf(rate[outcome.yaw.axis]);

    if (outcome.tiltDeg > MAX_TILT_DEG || spinOffAxisDeg > MAX_TILT_DEG) {
        outcome.verdict = Verdict::Tilted;
    } else if (outcome.yaw.axis != outcome.up.axis || outcome.yaw.sign != outcome.up.sign) {
        outcome.verdict = Verdict::Inconsistent;
    } else {
        outcome.verdict = Verdict::Ok;
    }
}

const char* ImuOrientationCheck::verdictName(Verdict verdict) {
    switch (verdict) {
        case Verdict::Running:
            return "running";
        case Verdict::Ok:
            return "ok";
        case Verdict::NoData:
            return "no_data";
        case Verdict::Moving:
            return "moving";
        case Verdict::NoGravity:
            return "no_gravity";
        case Verdict::Tilted:
            return "tilted";
        case Verdict::NoSpin:
            return "no_spin";
        case Verdict::Inconsistent:
            return "inconsistent";
        case Verdict::Aborted:
            return "aborted";
    }
    return "unknown";
}
