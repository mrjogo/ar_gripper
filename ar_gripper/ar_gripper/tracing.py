"""Timestamped trace of every Feetech bus transaction, for motion profiling.

The driver's own `/joint_states` runs at 5 Hz (`STATUS_UPDATE_INTERVAL_S`),
which is too coarse to say anything about a move that takes about a second: a
full 0.05 m stroke would be eight samples. This records the transactions the
driver is *already* performing, each with the monotonic time it was issued and
the time the reply landed, so a move can be reconstructed at the rate the bus
actually runs rather than at the rate the node happens to publish.

Off by default, and off means the original code path. The hook in
`feetech.FeetechServo._send_instruction` is one module-attribute read compared
against None; there is no buffer, no clock read and no branch taken beyond that
until someone calls `enable()`.

Enabled, it is deliberately cheap **because it is measuring time and must not
change it**. Per transaction it costs two `perf_counter()` calls and one tuple
append onto a plain list -- no formatting, no serialisation, no I/O, no lock of
its own (the append happens inside the mutex `_send_instruction` already holds,
so it is already serialised). Everything expensive happens in `write_csv()`,
after the motion is over. Measured overhead is in `test_tracing.py`; it is
around a microsecond against a bus transaction of well over a millisecond.

What this does NOT do is add traffic. It cannot make the picture denser than
the driver's own read cadence -- during `Gripper._wait_for_stop` that is one
position read per `_WAIT_CHECK_TIME_S` (0.1 s). To sample faster you have to
issue extra reads, and each one occupies the bus for its own round trip and
competes with the control loop; that is a different tool with a different
trade-off, and it is why it is not folded in here.

Usage, from anywhere holding a servo (or from the node's `bus_trace_path`
parameter, which does this for you):

    from ar_gripper import tracing

    tracing.enable()
    ...                                  # drive the gripper
    trace = tracing.disable()
    tracing.write_csv("/tmp/trace.csv", trace)

`scripts/analyze_bus_trace.py` turns the CSV into stroke times, cruise velocity
and an acceleration estimate.
"""

import csv
import os
import threading

# The active trace, or None. Read on every bus transaction, so it stays a plain
# module attribute: no property, no function call, no lock.
_TRACE = None

# Feetech instruction opcodes, as `_send_instruction` receives them.
OP_READ = 0x02
OP_WRITE = 0x03

# Register this module knows how to name, purely for the CSV's benefit.
_REGISTER_NAMES = {
    0x28: "torque_switch",
    0x29: "acceleration",
    0x2A: "goal_position",
    0x2E: "drive_speed",
    0x30: "torque_limit",
    0x38: "present_position",
    0x3A: "present_voltage",
    0x3C: "present_load",
    0x42: "moving_sign",
    0x45: "present_current",
}


class BusTrace:
    """A list of transactions. Deliberately not much more than that.

    Each event is a tuple, not an object: constructing a class instance per
    transaction would cost more than the measurement is worth, and the fields
    are fixed. `write_csv` is what gives them names.
    """

    __slots__ = ("events", "started_at")

    def __init__(self):
        # (t_issued, t_replied, servo_id, opcode, address, payload_tuple)
        self.events = []
        self.started_at = None

    def __len__(self):
        return len(self.events)


def enable(trace=None):
    """Start recording. Returns the trace being written to."""
    global _TRACE
    _TRACE = trace if trace is not None else BusTrace()
    return _TRACE


def disable():
    """Stop recording and return what was collected (None if it was not on)."""
    global _TRACE
    trace, _TRACE = _TRACE, None
    return trace


def active():
    """The trace currently recording, or None."""
    return _TRACE


def register_name(address):
    return _REGISTER_NAMES.get(address, f"0x{address:02X}")


def write_csv(path, trace):
    """Write a trace out. Called after the motion, never during it.

    `duration_s` is the bus round trip -- issue to reply -- which is what makes
    a stretched transaction visible as something other than a gap in the
    timeline.
    """
    if trace is None or not trace.events:
        raise ValueError("no trace to write (was tracing enabled?)")
    directory = os.path.dirname(os.path.abspath(path))
    if directory:
        os.makedirs(directory, exist_ok=True)
    t0 = trace.events[0][0]
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
        for issued, replied, servo_id, opcode, address, payload in trace.events:
            writer.writerow(
                [
                    f"{issued - t0:.6f}",
                    f"{replied - t0:.6f}",
                    f"{replied - issued:.6f}",
                    servo_id,
                    "read" if opcode == OP_READ else "write",
                    f"0x{address:02X}",
                    register_name(address),
                    " ".join(str(b) for b in payload),
                ]
            )
    return path


class PositionSampler:
    """Extra position reads, at a chosen rate, on their own thread.

    SEPARATE from tracing on purpose, and not enabled with it. The tracer is
    passive; this is not. Every sample is a real bus transaction that occupies
    the mutex `_send_instruction` holds, so at 115200 baud each one costs the
    control loop something on the order of a millisecond of bus time. Sampling
    at 100 Hz therefore spends roughly a tenth of the bus, and a move profiled
    this way is not quite the move the driver performs unobserved.

    Use it when resolution matters more than fidelity, and read the result
    against an unsampled run before trusting a number. The samples appear in
    the trace like any other transaction, so the driver's own read cadence
    stays visible next to them and a stretched control loop is not invisible.
    """

    def __init__(self, servo, rate_hz):
        if rate_hz <= 0:
            raise ValueError("rate_hz must be positive")
        self._servo = servo
        self._period = 1.0 / rate_hz
        self._stop = threading.Event()
        self._thread = None

    def start(self):
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        return self

    def _run(self):
        while not self._stop.wait(self._period):
            try:
                _ = self._servo.present_position
            except Exception:  # noqa: BLE001, S110 - see below
                # Deliberately blind and deliberately silent. A diagnostic
                # sampler must never be the reason a move fails, and it must not
                # log per sample either: at 100 Hz a logging call in this loop
                # would cost more than the read it is guarding. A dropped sample
                # shows up as a gap in the trace, which is the report.
                pass

    def stop(self, timeout=2.0):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=timeout)
            self._thread = None
