"""ARGripperStandalone: ROS-free API + behavior-preservation tests.

Covers prompt test levels:
* 1 unit maps      -> exact equality vs golden captured from the ROS node
* 2 wire traces    -> byte-identical to the golden Gripper operations
* 3 persistence    -> json schema + "avoid a rehome when a saved value is valid"
* 5 ROS-free import -> importable with rclpy (and all ROS msgs) blocked

Plus the new clean API (get_position / get_state / set_goal / write_goal_ticks /
open / close / release / halt / calibrate / is_calibrated).
"""

import json
import subprocess
import sys

import pytest
from conftest import FIXTURES_DIR

from ar_gripper.standalone import ARGripperStandalone

MAPS = json.loads((FIXTURES_DIR / "unit_maps_golden.json").read_text())
WIRE = json.loads((FIXTURES_DIR / "wire_traces_golden.json").read_text())
SID = WIRE["servo_id"]


def _hex(trace):
    return [packet.hex() for packet in trace]


# --------------------------------------------------------------------------- #
# Level 1: unit maps -- bit-for-bit identical to the original ROS-node classmethods
# --------------------------------------------------------------------------- #
def test_map_constants_match_node():
    for name, value in MAPS["constants"].items():
        assert getattr(ARGripperStandalone, name) == value


@pytest.mark.parametrize(
    "map_name",
    [
        "percent_to_stroke",
        "stroke_to_percent",
        "percent_to_effort",
        "effort_to_percent",
    ],
)
def test_map_values_bit_identical(map_name):
    fn = getattr(ARGripperStandalone, map_name)
    for arg, expected in MAPS[map_name]:
        result = fn(arg)
        assert result == expected, f"{map_name}({arg!r}) = {result!r} != {expected!r}"
        assert type(result) is type(expected)


def test_map_endpoints_exact():
    assert ARGripperStandalone.percent_to_stroke(0.0) == 0.0
    assert ARGripperStandalone.percent_to_stroke(100.0) == 0.05
    assert ARGripperStandalone.stroke_to_percent(0.0) == 0.0
    assert ARGripperStandalone.stroke_to_percent(0.05) == 100.0
    assert ARGripperStandalone.percent_to_effort(0.0) == 0.0
    assert ARGripperStandalone.percent_to_effort(100.0) == 1041.25
    assert ARGripperStandalone.effort_to_percent(0.0) == 0.0
    assert ARGripperStandalone.effort_to_percent(1041.25) == 100.0


def test_map_round_trip_inverses():
    for pct in (0.0, 25.0, 50.0, 100.0):
        assert ARGripperStandalone.stroke_to_percent(
            ARGripperStandalone.percent_to_stroke(pct)
        ) == pytest.approx(pct)
    for pct in (0.0, 25.0, 50.0, 100.0):
        assert ARGripperStandalone.effort_to_percent(
            ARGripperStandalone.percent_to_effort(pct)
        ) == pytest.approx(pct)


# --------------------------------------------------------------------------- #
# New clean API: reads
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize(
    "ticks, pct, meters",
    [(150, 100.0, 0.05), (4095, 0.0, 0.0), (2122, 50.0, 0.025)],
)
def test_get_position_units(make_standalone, ticks, pct, meters):
    sa, _fake, _path = make_standalone(present_position=ticks, saved_position=ticks)
    assert sa.get_position("ticks") == ticks
    assert sa.get_position("percent") == pct
    assert sa.get_position("m") == meters


def test_get_position_invalid_unit(make_standalone):
    sa, _fake, _path = make_standalone()
    with pytest.raises(ValueError):
        sa.get_position("furlongs")


def test_get_state_snapshot(make_standalone):
    sa, fake, _path = make_standalone(present_position=150, saved_position=150)
    fake.servo(SID).load_sequence = [0.0]
    fake.servo(SID).moving = 0
    state = sa.get_state()
    assert state == {
        "position_m": 0.05,
        "position_pct": 100.0,
        "ticks": 150,
        "load": 0.0,
        "effort_N": 0.0,
        "temperature": 25,
        "calibrated": True,
        "moving": False,
    }


def test_get_state_moving_flag(make_standalone):
    sa, fake, _path = make_standalone()
    fake.servo(SID).moving = 1
    assert sa.get_state()["moving"] is True


# --------------------------------------------------------------------------- #
# Level 2: wire traces byte-identical to the golden Gripper operations
# --------------------------------------------------------------------------- #
def test_open_wire_identical(make_standalone):
    sa, fake, _path = make_standalone()
    fake.trace.clear()
    sa.open()
    assert _hex(fake.trace) == WIRE["ops"]["open"]


def test_release_wire_identical(make_standalone):
    sa, fake, _path = make_standalone()
    fake.trace.clear()
    sa.release()
    assert _hex(fake.trace) == WIRE["ops"]["release"]


def test_halt_wire_identical(make_standalone):
    sa, fake, _path = make_standalone()
    fake.trace.clear()
    sa.halt()
    assert _hex(fake.trace) == WIRE["ops"]["halt"]


def test_write_goal_ticks_wire_identical(make_standalone):
    sa, fake, _path = make_standalone()
    fake.trace.clear()
    sa.write_goal_ticks(2048)
    assert _hex(fake.trace) == WIRE["ops"]["write_goal_2048"]


def test_set_goal_open_equivalent_to_open(make_standalone):
    """set_goal to fully-open with full torque drives the same packets as open()."""
    sa, fake, _path = make_standalone()
    fake.trace.clear()
    sa.set_goal(100.0, unit="percent")  # effort None -> full torque
    assert _hex(fake.trace) == WIRE["ops"]["open"]


# --------------------------------------------------------------------------- #
# New clean API: goal arg mapping (spy on the underlying Gripper)
# --------------------------------------------------------------------------- #
def _spy_goto(sa):
    calls = []
    sa.gripper.goto_position = lambda *a, **k: calls.append((a, k)) or True
    return calls


@pytest.mark.parametrize(
    "pos, unit, effort, exp_pct, exp_closing",
    [
        (0.05, "m", None, 100.0, 100),  # None effort -> MAX_TORQUE
        (50, "percent", 520.625, 50, 50.0),  # 520.625 N -> 50%
        (0.025, "m", 1041.25, 50.0, 100.0),  # full effort -> 100%
        (2122, "ticks", None, 50.0, 100),
    ],
)
def test_set_goal_blocking_maps_to_goto(
    make_standalone, pos, unit, effort, exp_pct, exp_closing
):
    sa, _fake, _path = make_standalone()
    calls = _spy_goto(sa)
    assert sa.set_goal(pos, unit=unit, effort=effort, blocking=True) is True
    (args, kwargs) = calls[0]
    assert args[0] == pytest.approx(exp_pct)
    assert args[1] == pytest.approx(exp_closing)
    assert kwargs["holding_torque"] == sa.holding_torque


def test_set_goal_invalid_unit(make_standalone):
    sa, _fake, _path = make_standalone()
    with pytest.raises(ValueError):
        sa.set_goal(1.0, unit="furlongs")


def test_set_goal_non_blocking_writes_goal_and_torque(make_standalone):
    sa, fake, _path = make_standalone()
    sa.set_goal(50, unit="percent", effort=1041.25, blocking=False)  # closing 100%
    # Fake mirrors goal_position onto present_position, proving the goal was sent.
    assert sa.get_position("ticks") == 2122
    # torque_limit (0x30) set to 100% -> raw int(100*10)=1000 -> little-endian E8 03
    assert bytes(fake.servo(SID).reg[0x30:0x32]) == bytes([0xE8, 0x03])


def test_close_maps_to_full_close(make_standalone):
    sa, _fake, _path = make_standalone()
    calls = _spy_goto(sa)
    sa.close()
    (args, kwargs) = calls[0]
    assert args[0] == 0
    assert args[1] == sa.gripper.MAX_TORQUE
    assert kwargs["holding_torque"] == sa.holding_torque


# --------------------------------------------------------------------------- #
# calibrate + is_calibrated
# --------------------------------------------------------------------------- #
def test_calibrate_wire_identical_and_persists(make_standalone):
    sa, fake, path = make_standalone(load_sequence=[5, 15, 15, 0])
    fake.trace.clear()
    assert sa.calibrate() is True
    # calibrate() delegates to Gripper.calibrate() (golden), then persists the new
    # encoder position (one present_position read via get_servo_position()).
    assert (
        _hex(fake.trace) == WIRE["ops"]["calibrate"] + WIRE["reads"]["present_position"]
    )
    assert json.loads(path.read_text()) == {"position": 150}


def test_is_calibrated_reflects_gripper(make_standalone):
    sa, _fake, _path = make_standalone()
    assert sa.is_calibrated is True
    sa.gripper._calibrated = False
    assert sa.is_calibrated is False


# --------------------------------------------------------------------------- #
# Level 3: persistence round-trip
# --------------------------------------------------------------------------- #
def test_avoid_rehome_when_saved_valid(make_standalone):
    """A valid saved position => verify_calibrated passes, no physical rehome."""
    sa, fake, _path = make_standalone(saved_position=150, present_position=150)
    assert sa.is_calibrated is True
    # Construction = handshake + _init_servo + exactly one verify present_position
    # read; no calibration motion, no re-save.
    assert _hex(fake.trace) == WIRE["construct"] + WIRE["reads"]["present_position"]


def test_rehome_and_persist_when_no_saved_file(make_standalone, tmp_path):
    missing = tmp_path / "does_not_exist.json"
    sa, _fake, path = make_standalone(
        saved_position=None, load_sequence=[5, 15, 15, 0], path=missing
    )
    assert sa.is_calibrated is True
    assert json.loads(path.read_text()) == {"position": 150}


def test_saved_json_schema_is_position_key(make_standalone):
    sa, _fake, path = make_standalone()
    sa.save_position()
    data = json.loads(path.read_text())
    assert list(data.keys()) == [ARGripperStandalone.SAVED_POSITION_KEY]
    assert isinstance(data["position"], int)


def test_corrupt_json_triggers_rehome(make_standalone, tmp_path):
    corrupt = tmp_path / "corrupt.json"
    corrupt.write_text("{not valid json")
    sa, _fake, path = make_standalone(
        saved_position=None, load_sequence=[5, 15, 15, 0], path=corrupt
    )
    assert sa.is_calibrated is True
    assert json.loads(path.read_text()) == {"position": 150}


def test_none_path_skips_persistence(fake_serials, fast_clock):
    """servo_position_path=None: no file I/O, still rehomes on construct."""
    from ar_gripper.feetech import USB2FeetechDevice

    device = USB2FeetechDevice("/dev/fake")
    fake = fake_serials[-1]
    fake.servo(SID).load_sequence = [5, 15, 15, 0]
    sa = ARGripperStandalone(
        servo_id=SID, name="primary", device=device, servo_position_path=None
    )
    assert sa.is_calibrated is True
    # No exception, no file — save_position is a no-op with no path.
    sa.save_position()


def test_calibrate_on_init_false_skips_flow(fake_serials, fast_clock):
    """Opt-in read-only-ish connect: no persistence read, no physical rehome."""
    from ar_gripper.feetech import USB2FeetechDevice

    device = USB2FeetechDevice("/dev/fake")
    fake = fake_serials[-1]
    sa = ARGripperStandalone(
        servo_id=SID,
        name="primary",
        device=device,
        servo_position_path=None,
        calibrate_on_init=False,
    )
    assert sa.is_calibrated is False
    # Only the servo handshake + _init_servo ran; no verify read, no calibration.
    assert _hex(fake.trace) == WIRE["construct"]


# --------------------------------------------------------------------------- #
# Construction variants + holding torque
# --------------------------------------------------------------------------- #
def test_builds_own_device_from_port(fake_serials, fast_clock, tmp_path):
    """device=None opens a USB2FeetechDevice from the port (LeRobot's path)."""
    path = tmp_path / "p.json"
    path.write_text(json.dumps({"position": 150}))
    sa = ARGripperStandalone(
        port="/dev/fake", servo_id=SID, name="primary", servo_position_path=str(path)
    )
    assert sa.is_calibrated is True
    assert fake_serials[-1].url == "/dev/fake"


def test_calibration_failure_raises(make_standalone):
    from ar_gripper.gripper import CalibrationError

    with pytest.raises(CalibrationError):
        make_standalone(saved_position=None, load_sequence=[0.0])


def test_holding_torque_default_and_set(make_standalone):
    sa, _fake, _path = make_standalone()
    assert sa.holding_torque == sa.gripper.HOLDING_TORQUE
    sa.holding_torque = 5
    assert sa.holding_torque == 5


def test_holding_torque_validation(make_standalone):
    sa, _fake, _path = make_standalone()
    with pytest.raises(ValueError):
        sa.holding_torque = -1
    with pytest.raises(ValueError):
        sa.holding_torque = sa.gripper.OVERLOAD_TORQUE


# --------------------------------------------------------------------------- #
# Level 5: ROS-free import guard
# --------------------------------------------------------------------------- #
def test_importable_with_ros_blocked():
    pkg_parent = str(FIXTURES_DIR.parents[1])
    code = (
        "import sys\n"
        "for m in ['rclpy', 'rcl_interfaces', 'ament_index_python', 'control_msgs',\n"
        "          'ar_gripper_interfaces', 'diagnostic_msgs', 'sensor_msgs', 'std_srvs']:\n"
        "    sys.modules[m] = None\n"
        "import ar_gripper.standalone\n"
        "from ar_gripper.standalone import ARGripperStandalone\n"
        "assert ARGripperStandalone.percent_to_stroke(100.0) == 0.05\n"
        "print('ROS_FREE_OK')\n"
    )
    result = subprocess.run(
        [sys.executable, "-c", code],
        capture_output=True,
        text=True,
        env={"PYTHONPATH": pkg_parent, "PATH": ""},
    )
    assert result.returncode == 0, result.stderr
    assert "ROS_FREE_OK" in result.stdout
