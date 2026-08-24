#!/usr/bin/env python3
"""Turn a Feetech bus trace into stroke times, velocity and acceleration.

Reads the CSV `ar_gripper.tracing.write_csv` produces and reports, per detected
move, how long it took and how fast the finger was going. Pure standard
library, no ROS: it runs on a trace copied off the robot.

    ./analyze_bus_trace.py /tmp/trace.csv
    ./analyze_bus_trace.py /tmp/trace.csv --csv /tmp/samples.csv

**How velocity is obtained, and why not from the endpoints.** Dividing stroke
by total move time understates the speed, because the total includes the ramps
at both ends and, for an action round trip, the driver's inrush window and its
wait-for-stop dwell. The cruise speed is instead fitted to the middle of the
move -- the samples between `--cruise-margin` of the travelled distance at each
end -- which is the part where the servo is not accelerating.

**Acceleration** is then `v_cruise / t_ramp`, with `t_ramp` taken from the time
the finger needs to reach cruise at the start of the move. For a trapezoidal
profile total time is `d/v + v/a`, so this can be cross-checked against a set
of moves of different lengths: regress total time on distance and the intercept
is `v/a`. Both are reported when there is enough data, because agreeing is the
evidence and disagreeing is the finding.

The tick-to-metre scale is the mechanism's: 3945 ticks over the 0.05 m stroke.
"""

import argparse
import csv
import itertools
import statistics
import sys

TICKS_PER_METRE = 78_900.0
PRESENT_POSITION = "present_position"
GOAL_POSITION = "goal_position"
# Below this the finger counts as stationary. One tick is 12.7 um, so this is
# just under two ticks -- tight enough to catch the ends of a move, loose
# enough not to trip on encoder dither.
MOVING_EPSILON_M = 2.5e-5
# Fraction of cruise speed that counts as "done accelerating". Below 1.0 so
# that quantisation noise near the plateau does not push the crossing late.
RAMP_FRACTION = 0.9


def decode_position(payload):
    """Two little-endian bytes, sign-magnitude in the top bit, to metres."""
    parts = [int(p) for p in payload.split()] if payload else []
    if len(parts) < 2:
        return None
    ticks = parts[0] | (parts[1] & 0x7F) << 8
    if parts[1] & 0x80:
        ticks = -ticks
    return ticks / TICKS_PER_METRE


def load_samples(path):
    """(t, position_m) for every present_position reply in the trace."""
    samples, goals = [], []
    with open(path, newline="") as handle:
        for row in csv.DictReader(handle):
            t = float(row["t_replied_s"])
            if row["register"] == PRESENT_POSITION and row["op"] == "read":
                position = decode_position(row["payload"])
                if position is not None:
                    samples.append((t, position))
            elif row["register"] == GOAL_POSITION and row["op"] == "write":
                goals.append(t)
    return samples, goals


def split_moves(samples, goals, idle_gap_s=0.30):
    """Group samples into moves.

    Segmented on `goal_position` WRITES when the trace has them, because that
    is the event that actually starts a move; the alternative -- inferring
    boundaries from the finger going quiet -- needs samples to exist during the
    idle periods, which is only true while something happens to be polling.
    Each window runs from one goal write to the next, with trailing stationary
    samples trimmed so a move's duration is motion and not the dwell after it.

    Falls back to idle detection for traces with no goal writes.
    """
    if goals:
        moves = []
        bounds = list(goals) + [float("inf")]
        for start_t, next_t in itertools.pairwise(bounds):
            window = [(t, p) for t, p in samples if start_t <= t < next_t]
            if len(window) < 4:
                continue
            # Trim the tail: everything after the finger last actually moved.
            last = len(window) - 1
            while (
                last > 0
                and abs(window[last][1] - window[last - 1][1]) <= MOVING_EPSILON_M
            ):
                last -= 1
            window = window[: last + 1]
            if (
                len(window) >= 4
                and abs(window[-1][1] - window[0][1]) > MOVING_EPSILON_M
            ):
                moves.append(window)
        return moves

    moves, current, last_moved_at = [], [], None
    for index, (t, position) in enumerate(samples):
        if index == 0:
            current = [(t, position)]
            continue
        moving = abs(position - samples[index - 1][1]) > MOVING_EPSILON_M
        if moving:
            last_moved_at = t
            current.append((t, position))
        elif last_moved_at is not None and t - last_moved_at > idle_gap_s:
            if len(current) >= 4:
                moves.append(current)
            current, last_moved_at = [], None
        elif current:
            current.append((t, position))
    if len(current) >= 4:
        moves.append(current)
    return moves


def analyse_move(move, cruise_margin=0.2):
    """Duration, distance, cruise velocity and an acceleration estimate."""
    positions = [p for _, p in move]
    start, end = positions[0], positions[-1]
    distance = abs(end - start)
    duration = move[-1][0] - move[0][0]
    if distance <= 0.0:
        return None

    lo = start + (end - start) * cruise_margin
    hi = start + (end - start) * (1.0 - cruise_margin)
    band = [(t, p) for t, p in move if min(lo, hi) <= p <= max(lo, hi)]

    cruise = None
    if len(band) >= 3:
        span_t = band[-1][0] - band[0][0]
        if span_t > 0:
            cruise = abs(band[-1][1] - band[0][1]) / span_t

    # Ramp time, taken from the VELOCITY trace rather than from a fraction of
    # the distance. The distance-fraction shortcut is wrong and quietly so: at
    # 31 mm/s and 250 mm/s2 the ramp covers 1.9 mm, under 4% of a 50 mm stroke,
    # so any margin big enough to be robust is already deep into cruise and the
    # acceleration comes out low -- and low by an amount that depends on the
    # stroke, which is why it looked like scatter rather than bias.
    #
    # Instead: differentiate, smooth over a 3-sample window against encoder
    # quantisation, and take the first time the finger reaches RAMP_FRACTION of
    # cruise. For a trapezoid that time is v/a.
    accel = None
    if cruise is not None and cruise > 0 and len(move) >= 4:
        speeds = []
        for (t0, p0), (t1, p1) in itertools.pairwise(move):
            dt = t1 - t0
            speeds.append(((t0 + t1) / 2.0, abs(p1 - p0) / dt if dt > 0 else 0.0))
        smoothed = [
            (
                speeds[i][0],
                sum(v for _, v in speeds[max(0, i - 1) : i + 2])
                / len(speeds[max(0, i - 1) : i + 2]),
            )
            for i in range(len(speeds))
        ]
        for t, v in smoothed:
            if v >= RAMP_FRACTION * cruise:
                ramp = t - move[0][0]
                if ramp > 0:
                    # The crossing is at RAMP_FRACTION of cruise, not at cruise,
                    # so the ramp measured here is RAMP_FRACTION * v/a. Scaling
                    # by it removes a bias that is otherwise a flat 1/0.9 = 11%
                    # overestimate -- systematic, so it would have looked like a
                    # confident answer rather than a wrong one.
                    accel = RAMP_FRACTION * cruise / ramp
                break
    return {
        "t_start": move[0][0],
        "duration_s": duration,
        "distance_m": distance,
        "mean_speed_mps": distance / duration if duration else 0.0,
        "cruise_mps": cruise,
        "accel_mps2": accel,
        "samples": len(move),
    }


def regress(moves):
    """Fit total time against distance: t = d/v + v/a. Returns (v, a)."""
    points = [(m["distance_m"], m["duration_s"]) for m in moves if m]
    if len(points) < 3:
        return None, None
    n = len(points)
    mx = sum(d for d, _ in points) / n
    my = sum(t for _, t in points) / n
    denom = sum((d - mx) ** 2 for d, _ in points)
    if denom == 0:
        return None, None
    slope = sum((d - mx) * (t - my) for d, t in points) / denom
    intercept = my - slope * mx
    if slope <= 0:
        return None, None
    velocity = 1.0 / slope
    accel = velocity / intercept if intercept > 0 else None
    return velocity, accel


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trace", help="CSV from ar_gripper.tracing.write_csv")
    parser.add_argument(
        "--cruise-margin",
        type=float,
        default=0.2,
        help="fraction of the stroke trimmed at each end before fitting cruise "
        "speed (default: 0.2)",
    )
    parser.add_argument("--csv", help="also write the extracted samples here")
    args = parser.parse_args()

    samples, goals = load_samples(args.trace)
    if not samples:
        print(
            "no present_position reads in this trace -- the driver only reads "
            "position while waiting for a move to stop, so a trace with none "
            "captured no motion",
            file=sys.stderr,
        )
        return 1

    span = samples[-1][0] - samples[0][0]
    rate = (len(samples) - 1) / span if span > 0 else 0.0
    print(f"{len(samples)} position samples over {span:.2f}s ({rate:.1f} Hz mean)")
    print(f"{len(goals)} goal_position writes")

    moves = [analyse_move(m, args.cruise_margin) for m in split_moves(samples, goals)]
    moves = [m for m in moves if m]
    if not moves:
        print("no moves detected", file=sys.stderr)
        return 1

    print()
    print(
        f"{'#':>2} {'start_s':>8} {'dist_mm':>8} {'dur_s':>7} "
        f"{'mean_mm/s':>10} {'cruise_mm/s':>12} {'accel_mm/s2':>12} {'n':>4}"
    )
    for index, m in enumerate(moves, 1):
        cruise = f"{m['cruise_mps'] * 1000:.1f}" if m["cruise_mps"] else "-"
        accel = f"{m['accel_mps2'] * 1000:.0f}" if m["accel_mps2"] else "-"
        print(
            f"{index:>2} {m['t_start']:>8.2f} {m['distance_m'] * 1000:>8.2f} "
            f"{m['duration_s']:>7.3f} {m['mean_speed_mps'] * 1000:>10.1f} "
            f"{cruise:>12} {accel:>12} {m['samples']:>4}"
        )

    cruises = [m["cruise_mps"] for m in moves if m["cruise_mps"]]
    if len(cruises) >= 2:
        print()
        print(
            f"cruise velocity: mean {statistics.mean(cruises) * 1000:.2f} mm/s, "
            f"sd {statistics.stdev(cruises) * 1000:.2f}, n={len(cruises)}"
        )
    accels = [m["accel_mps2"] for m in moves if m["accel_mps2"]]
    if len(accels) >= 2:
        print(
            f"acceleration:    mean {statistics.mean(accels) * 1000:.0f} mm/s2, "
            f"sd {statistics.stdev(accels) * 1000:.0f}, n={len(accels)}"
        )

    velocity, accel = regress(moves)
    if velocity:
        line = f"regression on {len(moves)} moves: v_max {velocity * 1000:.2f} mm/s"
        if accel:
            line += f", a {accel * 1000:.0f} mm/s2 (from the v/a intercept)"
        print(line)
        print(
            "  the regression and the per-move fits are independent; agreement "
            "is the evidence"
        )

    if args.csv:
        with open(args.csv, "w", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow(["t_s", "position_m"])
            writer.writerows(samples)
        print(f"\nsamples written to {args.csv}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
