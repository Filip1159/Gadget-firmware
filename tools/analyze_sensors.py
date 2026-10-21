#!/usr/bin/env python3
"""Turns sensor characterisation recordings (MAPPING.MD item 0.4) into bias / noise / range numbers.

Input: one or more CSV files recorded by the Gadget-UI "Sensor calibration" panel:

    target,distance_mm,sample,FRONT,FRONT_LEFT_45,...,BACK_RIGHT
    FRONT,300,1,298,OOR,...

`target` is the sensor that faced the wall (or ALL); only rows whose target is the sensor itself or
ALL are used for that sensor. Empty or OOR cells are out-of-range readings. `distance_mm` is the true
distance from the sensor to the wall, measured with a ruler.

Output: a Markdown table (paste into MAPPING.MD) and C++ arrays for lib/SensorCalibration/SensorCalibrationData.h.

Usage: python tools/analyze_sensors.py recording1.csv [recording2.csv ...]
"""
import csv
import statistics
import sys
from collections import defaultdict

SENSORS = [
    "FRONT", "FRONT_LEFT_45", "FRONT_LEFT", "FRONT_RIGHT_45", "FRONT_RIGHT",
    "BACK", "BACK_LEFT_45", "BACK_LEFT", "BACK_RIGHT_45", "BACK_RIGHT",
]

# A distance counts as trusted if, and only if, it and every shorter tested distance pass these.
MAX_OOR_RATE = 0.10
MAX_STD_MM = lambda distance: max(5.0, 0.03 * distance)  # noqa: E731


def parse_reading(cell):
    cell = cell.strip()
    if cell == "" or cell.upper() == "OOR":
        return None
    return int(cell)


def load(paths):
    """Returns {sensor: {distance_mm: [reading or None, ...]}}."""
    data = {sensor: defaultdict(list) for sensor in SENSORS}
    for path in paths:
        with open(path, newline="", encoding="utf-8") as handle:
            for row in csv.DictReader(handle):
                target = row["target"].strip().upper()
                distance = int(row["distance_mm"])
                for sensor in SENSORS:
                    if target in (sensor, "ALL"):
                        data[sensor][distance].append(parse_reading(row[sensor]))
    return data


def stats(distance, readings):
    valid = [r for r in readings if r is not None]
    oor_rate = 1 - len(valid) / len(readings)
    mean = statistics.fmean(valid) if valid else None
    return {
        "distance": distance,
        "samples": len(readings),
        "oor_rate": oor_rate,
        "mean": mean,
        "bias": mean - distance if mean is not None else None,
        "std": statistics.pstdev(valid) if len(valid) > 1 else (0.0 if valid else None),
        "min": min(valid) if valid else None,
        "max": max(valid) if valid else None,
    }


def trusted_range(rows):
    """Largest tested distance reached without a failing distance below it (0 = none)."""
    trusted = 0
    for row in rows:
        ok = row["mean"] is not None and row["oor_rate"] <= MAX_OOR_RATE and row["std"] <= MAX_STD_MM(row["distance"])
        if not ok:
            break
        trusted = row["distance"]
    return trusted


def fmt(value, spec="{:.1f}"):
    return "-" if value is None else spec.format(value)


def main(paths):
    data = load(paths)
    tested = [set(data[sensor]) for sensor in SENSORS if data[sensor]]
    if not tested:
        sys.exit("No usable rows in the input.")
    reference = sorted(tested[0])
    if any(sorted(t) != reference for t in tested):
        sys.exit("Every sensor needs the same set of distances; got: "
                 + ", ".join(f"{s}={sorted(data[s])}" for s in SENSORS if data[s]))

    curves, ranges = [], []
    print("| Sensor | Distance (mm) | Samples | OOR % | Mean (mm) | Bias (mm) | Std (mm) | Min | Max |")
    print("|---|---|---|---|---|---|---|---|---|")
    for sensor in SENSORS:
        rows = [stats(d, data[sensor][d]) for d in sorted(data[sensor])]
        for row in rows:
            print(
                f"| {sensor} | {row['distance']} | {row['samples']} | {row['oor_rate'] * 100:.0f} | {fmt(row['mean'])} | "
                f"{fmt(row['bias'], '{:+.1f}')} | {fmt(row['std'])} | {fmt(row['min'], '{}')} | {fmt(row['max'], '{}')} |"
            )
        if not rows or any(r["mean"] is None for r in rows):
            print(f"| {sensor} | no usable data: left uncorrected | | | | | | | |")
            curves.append(list(reference))  # identity curve
            ranges.append(0)
            continue
        curves.append([round(r["mean"]) for r in rows])
        ranges.append(trusted_range(rows))

    print("\nTrusted range = largest tested distance where it and every shorter one has "
          f"OOR <= {MAX_OOR_RATE * 100:.0f}% and std <= max(5 mm, 3% of distance).")
    print("The bias is NOT constant (it changes with distance), so the correction is a piecewise-linear curve through the")
    print("mean raw reading at each reference distance (see lib/SensorCalibration), not a single offset.\n")
    print("| Sensor | Trusted max range (mm) | " + " | ".join(f"raw @ {d}" for d in reference) + " |")
    print("|---|---|" + "---|" * len(reference))
    for sensor, curve, trusted in zip(SENSORS, curves, ranges):
        print(f"| {sensor} | {trusted if trusted else 'none'} | " + " | ".join(str(v) for v in curve) + " |")

    print("\n// lib/SensorCalibration/SensorCalibrationData.h (SensorDirection order)")
    print("constexpr uint8_t SENSOR_REF_COUNT = " + str(len(reference)) + ";")
    print("constexpr uint16_t SENSOR_REF_DISTANCES_MM[SENSOR_REF_COUNT] = {" + ", ".join(map(str, reference)) + "};")
    print("// Mean raw reading of each sensor at the reference distances.")
    print("constexpr uint16_t SENSOR_RAW_AT_REF_MM[10][SENSOR_REF_COUNT] = {")
    for sensor, curve in zip(SENSORS, curves):
        print("    {" + ", ".join(map(str, curve)) + "},  // " + sensor)
    print("};")
    print("constexpr uint16_t SENSOR_TRUSTED_MAX_MM[10] = {" + ", ".join(map(str, ranges)) + "};")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    main(sys.argv[1:])
