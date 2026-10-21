#pragma once

#include <Odometry.h>
#include <stddef.h>
#include <stdint.h>

#include <string>

// The WebSocket wire format (see CLAUDE.md), hardware-free so it can be unit tested on the host with
// `pio test -e native`. Encoders return the exact frame text; parsers are strict (all-or-nothing) and
// take the argument text after the command word.

// Reading value that means "no obstacle / out of range" (same value as OUT_OF_RANGE_MM).
constexpr uint16_t PROTOCOL_OUT_OF_RANGE = 0xFFFF;

// "POSE X=<mm> Y=<mm> TH=<mrad> T=<ms> Q=0". Q=1 will flag a suspected wheel slip (MAPPING.MD 1.5).
std::string formatPose(const Pose& pose, uint32_t tMs);

// "SWEEP_POSE T0=<ms> X0=<mm> Y0=<mm> TH0=<mrad> T1=<ms> X1=<mm> Y1=<mm> TH1=<mrad>"
std::string formatSweepPose(const StampedPose& start, const StampedPose& end);

// " FRONT=412 FRONT_LEFT_45=OOR ..." for `count` readings, `names[i]` being the wire name of reading i.
std::string formatSweep(const char* const names[], const uint16_t readings[], size_t count);

// "STATE SPEED=<n> P=<f> I=<f> D=<f> MEASURING=<0|1>"
std::string formatState(unsigned speed, float p, float i, float d, bool measuring);

// "CAL <distance_mm> <index>/<total>" + the sweep fields from formatSweep.
std::string formatCal(unsigned distanceMm, unsigned index, unsigned total, const std::string& sweepFields);

// "CAL_DONE <distance_mm> <done>"
std::string formatCalDone(unsigned distanceMm, unsigned done);

// Strict unsigned integer: optional surrounding whitespace, then digits only. Rejects empty, signed and
// non-numeric text. Values too large for `long` saturate instead of wrapping.
bool parseUint(const char* text, long& value);

// "<P> <I> <D>": exactly three finite floats >= 0.
bool parsePidArgs(const char* args, float& p, float& i, float& d);

// "<distance_mm> <count>": two unsigned integers, distance in 1..maxDistanceMm and count in 1..maxCount.
bool parseCalArgs(const char* args, long maxDistanceMm, long maxCount, long& distanceMm, long& count);
