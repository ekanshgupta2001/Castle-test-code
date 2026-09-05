#!/usr/bin/env python3
"""Summarise a MatchLogger CSV pulled off the robot.

The robot writes one row per control loop to /sdcard/FIRST/data/. Until now nothing read them,
which meant every practice run produced evidence nobody looked at. This turns a run into the four
answers you actually want afterwards:

  * did the loop keep up, or did it stall
  * was the follower tracking, or just moving
  * did the battery sag
  * what did each macro attempt actually do

Usage:
    python3 tools/analyze_log.py teleop_20260904_191500.csv
    python3 tools/analyze_log.py --plot teleop_*.csv       # needs matplotlib
    adb pull /sdcard/FIRST/data ./logs                     # to get them off the robot

Standard library only unless you pass --plot.
"""

import argparse
import csv
import math
import sys
from collections import Counter, OrderedDict


def parse_float(value):
    """CSV cells are strings; missing and NaN cells must not be mistaken for zero."""
    if value is None or value == "":
        return None
    try:
        f = float(value)
    except ValueError:
        return None
    return None if math.isnan(f) else f


def load(path):
    with open(path, newline="") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise SystemExit(f"{path}: no data rows")
    return rows


def column(rows, name):
    """Numeric values from one column, skipping blanks and NaNs."""
    if name not in rows[0]:
        return []
    return [v for v in (parse_float(r.get(name)) for r in rows) if v is not None]


def percentile(values, pct):
    if not values:
        return None
    ordered = sorted(values)
    k = (len(ordered) - 1) * pct / 100.0
    lower = math.floor(k)
    upper = math.ceil(k)
    if lower == upper:
        return ordered[int(k)]
    return ordered[lower] * (upper - k) + ordered[upper] * (k - lower)


def fmt(value, spec=".1f", suffix=""):
    return "n/a" if value is None else f"{value:{spec}}{suffix}"


def describe(rows, name, unit="", spec=".1f"):
    values = column(rows, name)
    if not values:
        return None
    return OrderedDict(
        min=fmt(min(values), spec, unit),
        mean=fmt(sum(values) / len(values), spec, unit),
        p95=fmt(percentile(values, 95), spec, unit),
        max=fmt(max(values), spec, unit),
    )


def section(title):
    print()
    print(title)
    print("-" * len(title))


def report_duration(rows):
    times = column(rows, "t_ms")
    section("Run")
    print(f"  rows      {len(rows)}")
    if times:
        seconds = (max(times) - min(times)) / 1000.0
        print(f"  duration  {seconds:.1f} s")
        if seconds > 0:
            print(f"  avg rate  {len(rows) / seconds:.1f} Hz")

    phases = Counter(r.get("phase", "") for r in rows if r.get("phase"))
    if phases:
        shown = ", ".join(f"{p}={c}" for p, c in phases.most_common())
        print(f"  phases    {shown}")


def report_loop(rows):
    values = column(rows, "loop_ms")
    if not values:
        return
    section("Loop timing")
    stats = describe(rows, "loop_ms", " ms")
    for key, value in stats.items():
        print(f"  {key:<9} {value}")

    for threshold in (30, 50, 100):
        spikes = sum(1 for v in values if v >= threshold)
        if spikes:
            pct = 100.0 * spikes / len(values)
            print(f"  over {threshold:>3} ms  {spikes} loops ({pct:.2f}%)")

    worst = max(values)
    if worst >= 100:
        print("  NOTE: loops over 100 ms mean the robot was blind and unresponsive that long.")


def report_battery(rows):
    values = column(rows, "battery_v")
    if not values:
        return
    section("Battery")
    print(f"  start     {values[0]:.2f} V")
    print(f"  end       {values[-1]:.2f} V")
    print(f"  minimum   {min(values):.2f} V")
    sag = max(values) - min(values)
    print(f"  sag       {sag:.2f} V")
    if min(values) < 11.0:
        print("  NOTE: dropped under 11 V - suspect this before suspecting the code.")


def report_tracking(rows):
    errors = column(rows, "trans_error")
    heading = column(rows, "heading_error")
    if not errors and not heading:
        return
    section("Path tracking (while following)")

    following = [r for r in rows if parse_float(r.get("path_busy")) == 1]
    print(f"  following {len(following)} of {len(rows)} loops")
    if not following:
        print("  (never followed a path - nothing to evaluate)")
        return

    for label, name, unit in (("translational", "trans_error", " in"),
                              ("heading", "heading_error", " rad")):
        stats = describe(following, name, unit, ".3f")
        if stats:
            line = "  ".join(f"{k}={v}" for k, v in stats.items())
            print(f"  {label:<13} {line}")


def report_macros(rows):
    section("Macros")
    # A macro is one contiguous run of rows sharing a name, so count transitions rather than rows.
    # The final outcome is whatever the last row of that run said - the earlier rows all say
    # RUNNING, and the rows after it have gone back to "idle" and must not overwrite it.
    attempts = []
    previous = None
    for row in rows:
        name = row.get("macro", "")
        if not name or name == "idle":
            previous = name
            continue
        if name != previous:
            attempts.append([name, row.get("macro_outcome", "")])
        elif attempts:
            attempts[-1][1] = row.get("macro_outcome", "") or attempts[-1][1]
        previous = name

    if not attempts:
        print("  none run")
        return

    outcomes = Counter(outcome for _, outcome in attempts)
    for name, outcome in attempts:
        print(f"  {name:<20} {outcome}")
    print()
    print("  " + ", ".join(f"{k}={v}" for k, v in outcomes.most_common()))


def report_intake(rows):
    modes = Counter(r.get("intake_mode", "") for r in rows if r.get("intake_mode"))
    if not modes:
        return
    section("Intake")
    total = sum(modes.values())
    for mode, count in modes.most_common():
        print(f"  {mode:<12} {100.0 * count / total:5.1f}%  ({count} loops)")

    unjam = sum(1 for r in rows if parse_float(r.get("unjamming")) == 1)
    if unjam:
        print(f"  unjamming    {unjam} loops")

    stats = describe(rows, "intake_amps", " A", ".2f")
    if stats:
        print("  current      " + "  ".join(f"{k}={v}" for k, v in stats.items()))


def report_localization(rows):
    results = Counter(r.get("localization", "") for r in rows if r.get("localization"))
    if not results:
        return
    section("Localization")
    total = sum(results.values())
    for result, count in results.most_common():
        print(f"  {result:<20} {100.0 * count / total:5.1f}%  ({count})")
    rejected = sum(c for r, c in results.items() if r.startswith("REJECTED"))
    if rejected > total * 0.25:
        print("  NOTE: over a quarter of vision fixes rejected - check the camera mount constants.")


def plot(rows, path):
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("\n--plot needs matplotlib:  pip install matplotlib", file=sys.stderr)
        return

    times = [(t or 0) / 1000.0 for t in (parse_float(r.get("t_ms")) for r in rows)]
    panels = [
        ("Loop time (ms)", column(rows, "loop_ms")),
        ("Battery (V)", column(rows, "battery_v")),
        ("Translational error (in)", column(rows, "trans_error")),
        ("Intake velocity (ticks/s)", column(rows, "intake_actual_v")),
    ]
    panels = [(title, values) for title, values in panels if values]
    if not panels:
        print("nothing plottable in this file", file=sys.stderr)
        return

    figure, axes = plt.subplots(len(panels), 1, sharex=True, figsize=(11, 2.2 * len(panels)))
    if len(panels) == 1:
        axes = [axes]
    for axis, (title, values) in zip(axes, panels):
        axis.plot(times[:len(values)], values, linewidth=0.9)
        axis.set_ylabel(title, fontsize=8)
        axis.grid(alpha=0.3)
    axes[-1].set_xlabel("time (s)")
    figure.suptitle(path)
    figure.tight_layout()
    plt.show()


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("logs", nargs="+", help="CSV files from /sdcard/FIRST/data")
    parser.add_argument("--plot", action="store_true", help="also draw time-series plots")
    args = parser.parse_args()

    for path in args.logs:
        print("=" * 70)
        print(path)
        print("=" * 70)
        rows = load(path)

        report_duration(rows)
        report_loop(rows)
        report_battery(rows)
        report_tracking(rows)
        report_intake(rows)
        report_localization(rows)
        report_macros(rows)
        print()

        if args.plot:
            plot(rows, path)


if __name__ == "__main__":
    main()
