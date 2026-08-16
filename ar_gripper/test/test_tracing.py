"""The bus tracer: correctness, and the two cost claims it makes.

The cost claims are the point of the last two tests. An instrument that
measures time has to be cheap enough not to be measuring itself, and one that
is off has to be genuinely off rather than merely quick.
"""

import csv
import importlib.util
import time
from pathlib import Path

import pytest
from ar_gripper import tracing
from ar_gripper.feetech import USB2FeetechDevice
from ar_gripper.mock import install_fake_serial


@pytest.fixture(autouse=True)
def _tracing_off():
    """No test may leak an enabled tracer into the next one."""
    tracing.disable()
    yield
    tracing.disable()


@pytest.fixture
def servo(monkeypatch):
    from ar_gripper.feetech import FeetechSMSServo

    install_fake_serial(monkeypatch)
    device = USB2FeetechDevice("/dev/fake")
    return FeetechSMSServo(device, 1)


def test_disabled_by_default_records_nothing(servo):
    assert tracing.active() is None
    servo.present_position
    assert tracing.active() is None
    assert tracing.disable() is None


def test_records_reads_and_writes_with_timestamps(servo):
    trace = tracing.enable()
    servo.present_position
    servo.goal_position = 2048
    assert tracing.disable() is trace

    assert len(trace) >= 2
    ops = {event[3] for event in trace.events}
    assert tracing.OP_READ in ops
    assert tracing.OP_WRITE in ops
    for issued, replied, servo_id, _op, _addr, _payload in trace.events:
        assert servo_id == 1
        assert replied >= issued, "reply cannot precede the request"


def test_addresses_are_recorded_and_named(servo):
    trace = tracing.enable()
    servo.present_position
    tracing.disable()
    addresses = {event[4] for event in trace.events}
    assert 0x38 in addresses
    assert tracing.register_name(0x38) == "present_position"
    assert tracing.register_name(0x99) == "0x99"


def test_write_csv_round_trips(servo, tmp_path):
    trace = tracing.enable()
    servo.present_position
    servo.present_position
    tracing.disable()

    path = tmp_path / "trace.csv"
    tracing.write_csv(str(path), trace)
    rows = list(csv.DictReader(path.open()))
    assert len(rows) == len(trace)
    assert rows[0]["register"] == "present_position"
    assert rows[0]["op"] == "read"
    # Times are relative to the first event, so a trace is comparable across runs.
    assert float(rows[0]["t_issued_s"]) == pytest.approx(0.0, abs=1e-9)
    assert float(rows[0]["duration_s"]) >= 0.0


def test_write_csv_refuses_an_empty_trace(tmp_path):
    with pytest.raises(ValueError, match="no trace"):
        tracing.write_csv(str(tmp_path / "x.csv"), tracing.BusTrace())


def test_disabled_costs_nothing_measurable(servo):
    """Off must be off: the guarded path may not be slower than no guard.

    Measured against the mock, where a transaction is microseconds rather than
    the milliseconds a real bus takes, so any overhead shows up far more
    starkly here than it ever could on hardware.
    """
    reps = 2000
    for _ in range(200):  # warm up
        servo.present_position

    start = time.perf_counter()
    for _ in range(reps):
        servo.present_position
    disabled = (time.perf_counter() - start) / reps

    tracing.enable()
    start = time.perf_counter()
    for _ in range(reps):
        servo.present_position
    enabled = (time.perf_counter() - start) / reps
    tracing.disable()

    overhead = enabled - disabled
    # A real transaction at 115200 baud is >1 ms. Anything under 50 us is
    # noise against that by a factor of 20+, and the mock exaggerates it.
    assert overhead < 50e-6, (
        f"tracing added {overhead * 1e6:.1f} us per transaction "
        f"({disabled * 1e6:.1f} us off, {enabled * 1e6:.1f} us on) -- too much "
        "for something whose job is measuring time"
    )


def test_enabled_does_not_change_what_the_bus_does(servo):
    """The trace must observe the conversation, not alter it."""
    servo.goal_position = 1500
    baseline = servo.present_position

    trace = tracing.enable()
    traced = servo.present_position
    tracing.disable()

    assert traced == baseline
    after = servo.present_position
    assert after == baseline
    assert len(trace) == 1, "tracing must not issue transactions of its own"


# -- the analyser, against a profile whose answers are known ---------------------
#
# These exist because the analyser's first two versions were confidently wrong:
# it merged five moves into one (it split on idle samples, which a trace need
# not contain), and it then read acceleration off a fraction of the DISTANCE,
# which is deep into cruise and gave a stroke-dependent underestimate that
# looked like scatter. Ground truth is the only thing that catches that class of
# error, so it is pinned here rather than eyeballed once.

_ANALYZER = (
    Path(__file__).resolve().parents[1]
    / "ar_gripper"
    / "scripts"
    / "analyze_bus_trace.py"
)


def _load_analyzer():
    spec = importlib.util.spec_from_file_location("_analyze_bus_trace", _ANALYZER)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _trapezoid(distance, velocity, accel, sample_hz):
    """(t, position) for a symmetric trapezoidal move."""
    t_ramp = velocity / accel
    d_ramp = 0.5 * accel * t_ramp**2
    triangular = 2 * d_ramp >= distance
    t_end = (
        2 * (distance / accel) ** 0.5
        if triangular
        else 2 * t_ramp + (distance - 2 * d_ramp) / velocity
    )
    points, t = [], 0.0
    while t <= t_end:
        if triangular:
            half = t_end / 2
            p = (
                0.5 * accel * t * t
                if t <= half
                else distance - 0.5 * accel * (t_end - t) ** 2
            )
        elif t < t_ramp:
            p = 0.5 * accel * t * t
        elif t <= t_end - t_ramp:
            p = d_ramp + velocity * (t - t_ramp)
        else:
            p = distance - 0.5 * accel * (t_end - t) ** 2
        points.append((t, min(p, distance)))
        t += 1.0 / sample_hz
    points.append((t_end, distance))
    return points


def _write_synthetic_trace(path, velocity, accel, distances, sample_hz=100.0):
    rows, clock = [], 0.0
    for distance in distances:
        profile = _trapezoid(distance, velocity, accel, sample_hz)
        rows.append((clock, clock + 0.001, "write", "0x2A", "goal_position", ""))
        for t, p in profile:
            ticks = int(round(p * 78_900.0))
            rows.append(
                (
                    clock + t,
                    clock + t + 0.0014,
                    "read",
                    "0x38",
                    "present_position",
                    f"{ticks & 0xFF} {(ticks >> 8) & 0x7F}",
                )
            )
        clock += profile[-1][0] + 1.0
    with open(path, "w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "t_issued_s",
                "t_replied_s",
                "duration_s",
                "servo_id",
                "op",
                "address",
                "register",
                "payload",
            ]
        )
        for issued, replied, op, addr, reg, payload in rows:
            writer.writerow(
                [
                    f"{issued:.6f}",
                    f"{replied:.6f}",
                    f"{replied - issued:.6f}",
                    1,
                    op,
                    addr,
                    reg,
                    payload,
                ]
            )


def test_analyser_recovers_a_known_profile(tmp_path):
    velocity, accel = 0.031, 0.25
    path = tmp_path / "synthetic.csv"
    _write_synthetic_trace(path, velocity, accel, [0.05, 0.04, 0.03, 0.02, 0.01])

    analyzer = _load_analyzer()
    samples, goals = analyzer.load_samples(str(path))
    moves = [analyzer.analyse_move(m) for m in analyzer.split_moves(samples, goals)]
    moves = [m for m in moves if m]

    assert len(moves) == 5, "one move per goal write"
    for expected, move in zip([0.05, 0.04, 0.03, 0.02, 0.01], moves):
        assert move["distance_m"] == pytest.approx(expected, abs=1e-4)
        assert move["cruise_mps"] == pytest.approx(velocity, rel=0.02)
        assert move["accel_mps2"] == pytest.approx(accel, rel=0.10)

    fitted_v, fitted_a = analyzer.regress(moves)
    assert fitted_v == pytest.approx(velocity, rel=0.02)
    assert fitted_a == pytest.approx(accel, rel=0.15)


def test_analyser_needs_no_idle_samples_between_moves(tmp_path):
    """The first splitter needed them and silently merged everything without."""
    path = tmp_path / "back_to_back.csv"
    _write_synthetic_trace(path, 0.031, 0.25, [0.05, 0.05, 0.05])
    analyzer = _load_analyzer()
    samples, goals = analyzer.load_samples(str(path))
    assert len(analyzer.split_moves(samples, goals)) == 3
