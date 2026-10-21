#pragma once

// Physical constants of the robot, used by odometry and mapping (see MAPPING.MD).
//
// Frame (ROS REP-103): origin = centre of the robot, +x forward, +y left, theta counter-clockwise.
// Lengths in millimetres, angles in radians. Every value says how it was obtained. VERIFIED
// values were confirmed by the user against the real robot; NOMINAL ones are datasheet/drawing values.

namespace RobotGeometry {

// ---- Wheels and encoders ----

// VERIFIED (user-confirmed): wheel diameter, mm.
constexpr float WHEEL_DIAMETER_MM = 80.0f;

// MEASURED: PCNT counts per wheel revolution (user-confirmed equal for both wheels).
// 8 encoder pulses per motor revolution * 1:120 gearbox = 960 per channel; the PCNT counts both
// edges of channel A (x2), so 1920. Hand-turned one revolution and read ~1920. See ENCODERS.MD.
constexpr float TICKS_PER_WHEEL_REV = 1920.0f;

// VERIFIED (user-confirmed): distance between the two wheel contact centres, mm. UI body is 135 mm
// wide with 16 mm wide wheels flush with the sides: 135 - 16 = 119.
// Refined later by the rotation calibration (item 1.4).
constexpr float TRACK_WIDTH_MM = 119.0f;

// Assumption (matches the UI drawing, wheels centred on the body): the wheel axle is at the body centre
// (robot frame origin). Re-check during the rotation calibration (1.4) if spins look off-centre.

// Derived.
constexpr float PI_F = 3.14159265f;
constexpr float WHEEL_CIRCUMFERENCE_MM = PI_F * WHEEL_DIAMETER_MM;
constexpr float TICKS_PER_MM = TICKS_PER_WHEEL_REV / WHEEL_CIRCUMFERENCE_MM;
constexpr float MM_PER_TICK = WHEEL_CIRCUMFERENCE_MM / TICKS_PER_WHEEL_REV;

// ---- Distance sensors ----

// NOMINAL (datasheet): VL53L0X beam cone, full angle (datasheet ~25 deg), radians.
constexpr float SENSOR_BEAM_ANGLE_RAD = 25.0f * PI_F / 180.0f;

struct SensorMount {
    float x;        // mm, forward of the robot centre
    float y;        // mm, left of the robot centre
    float bearing;  // rad, direction the sensor looks in, 0 = forward, counter-clockwise positive
};

// VERIFIED (user-confirmed against Gadget-UI `robot-geometry.ts` `SENSOR_MOUNTS`), converted with
// x_robot = -y_ui, y_robot = -x_ui, bearing = 1.5 pi - angle_ui. Body 180 x 135 mm, 18 mm corner
// chamfers, front/back sensors centred on the edge, corner sensors mid-chamfer, side sensors 35 mm
// from the front/back edge.
// Index = SensorDirection enum order: FRONT, FRONT_LEFT_45, FRONT_LEFT, FRONT_RIGHT_45, FRONT_RIGHT,
// BACK, BACK_LEFT_45, BACK_LEFT, BACK_RIGHT_45, BACK_RIGHT.
constexpr SensorMount SENSOR_MOUNTS[10] = {
    {90.0f, 0.0f, 0.0f},                     // FRONT
    {83.636f, 61.136f, 0.25f * PI_F},        // FRONT_LEFT_45
    {55.0f, 67.5f, 0.5f * PI_F},             // FRONT_LEFT
    {83.636f, -61.136f, -0.25f * PI_F},      // FRONT_RIGHT_45
    {55.0f, -67.5f, -0.5f * PI_F},           // FRONT_RIGHT
    {-90.0f, 0.0f, PI_F},                    // BACK
    {-83.636f, 61.136f, 0.75f * PI_F},       // BACK_LEFT_45
    {-55.0f, 67.5f, 0.5f * PI_F},            // BACK_LEFT
    {-83.636f, -61.136f, -0.75f * PI_F},     // BACK_RIGHT_45
    {-55.0f, -67.5f, -0.5f * PI_F},          // BACK_RIGHT
};

}  // namespace RobotGeometry
