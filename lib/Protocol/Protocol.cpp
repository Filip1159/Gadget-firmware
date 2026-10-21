#include "Protocol.h"

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

std::string formatPose(const Pose& pose, uint32_t tMs) {
    char buffer[80];
    snprintf(
        buffer, sizeof(buffer), "POSE X=%ld Y=%ld TH=%ld T=%lu Q=0", lroundf(pose.x), lroundf(pose.y),
        lroundf(pose.theta * 1000.0f), static_cast<unsigned long>(tMs)
    );
    return buffer;
}

std::string formatSweepPose(const StampedPose& start, const StampedPose& end) {
    char buffer[160];
    snprintf(
        buffer, sizeof(buffer), "SWEEP_POSE T0=%lu X0=%ld Y0=%ld TH0=%ld T1=%lu X1=%ld Y1=%ld TH1=%ld",
        static_cast<unsigned long>(start.tMs), lroundf(start.pose.x), lroundf(start.pose.y),
        lroundf(start.pose.theta * 1000.0f), static_cast<unsigned long>(end.tMs), lroundf(end.pose.x),
        lroundf(end.pose.y), lroundf(end.pose.theta * 1000.0f)
    );
    return buffer;
}

std::string formatSweep(const char* const names[], const uint16_t readings[], size_t count) {
    std::string fields;
    for (size_t i = 0; i < count; i++) {
        fields += ' ';
        fields += names[i];
        fields += '=';
        fields += (readings[i] == PROTOCOL_OUT_OF_RANGE) ? std::string("OOR") : std::to_string(readings[i]);
    }
    return fields;
}

std::string formatState(unsigned speed, float p, float i, float d, bool measuring) {
    char buffer[96];
    snprintf(
        buffer, sizeof(buffer), "STATE SPEED=%u P=%.4f I=%.4f D=%.4f MEASURING=%d", speed, p, i, d, measuring ? 1 : 0
    );
    return buffer;
}

std::string formatCal(unsigned distanceMm, unsigned index, unsigned total, const std::string& sweepFields) {
    return "CAL " + std::to_string(distanceMm) + " " + std::to_string(index) + "/" + std::to_string(total) +
           sweepFields;
}

std::string formatCalDone(unsigned distanceMm, unsigned done) {
    return "CAL_DONE " + std::to_string(distanceMm) + " " + std::to_string(done);
}

bool parseUint(const char* text, long& value) {
    while (isspace(static_cast<unsigned char>(*text))) {
        text++;
    }
    size_t length = strlen(text);
    while (length > 0 && isspace(static_cast<unsigned char>(text[length - 1]))) {
        length--;
    }
    if (length == 0) {
        return false;
    }
    for (size_t i = 0; i < length; i++) {
        if (!isdigit(static_cast<unsigned char>(text[i]))) {
            return false;
        }
    }
    value = strtol(text, nullptr, 10);  // saturates at LONG_MAX
    return true;
}

bool parsePidArgs(const char* args, float& p, float& i, float& d) {
    float a, b, c;
    char extra;
    if (sscanf(args, "%f %f %f %c", &a, &b, &c, &extra) != 3 || !isfinite(a) || !isfinite(b) || !isfinite(c) ||
        a < 0 || b < 0 || c < 0) {
        return false;
    }
    p = a;
    i = b;
    d = c;
    return true;
}

bool parseCalArgs(const char* args, long maxDistanceMm, long maxCount, long& distanceMm, long& count) {
    const char* space = strchr(args, ' ');
    if (space == nullptr) {
        return false;
    }
    long distance, number;
    if (!parseUint(std::string(args, space - args).c_str(), distance) || !parseUint(space + 1, number) ||
        distance < 1 || distance > maxDistanceMm || number < 1 || number > maxCount) {
        return false;
    }
    distanceMm = distance;
    count = number;
    return true;
}
