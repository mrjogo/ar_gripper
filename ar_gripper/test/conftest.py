"""Pytest fixtures for the hardware-free ar_gripper test suite.

The Feetech bus stand-ins live in the packaged ``ar_gripper.mock`` module (so
external consumers can loopback-test too). This module only wires them into
pytest and puts the package source tree on ``sys.path``.

The package under test lives at ``<repo>/ar_gripper/ar_gripper`` (an ament_cmake
package, so there is no editable install); we prepend that directory to
``sys.path`` here so ``import ar_gripper.*`` resolves to the source tree.

It also confines the suite's ROS traffic to a domain of its own, before any of
it starts. See the comment on that below: some of these tests build a real
driver, and a real driver COMMANDS THINGS.
"""

import os
import sys
from pathlib import Path

import pytest

# <repo>/ar_gripper/ar_gripper/test/conftest.py -> parents[1] == <repo>/ar_gripper/ar_gripper
_PKG_PARENT = Path(__file__).resolve().parents[1]
if str(_PKG_PARENT) not in sys.path:
    sys.path.insert(0, str(_PKG_PARENT))

# Confine this suite to a ROS domain of its own, and to this machine.
#
# Not a tidiness measure. Several tests build a real ARGripperNode, and a real
# driver publishes real joint commands: run this suite on the domain of a
# running simulator and it physically drives that simulator's finger (observed:
# 0.025 m -> 0.000 m), while its own reads get answered by the wrong publisher.
# Point it at real hardware and it is a unit-test run commanding the robot.
#
# Overridden rather than defaulted, because the ambient domain is exactly the
# hazard: inheriting a shell that happens to be pointed at a live stage is how
# this happens.
#
# 60-79 is chosen to clear every domain the integrating robot workspace
# actuates something on. As of this writing that is 42 (an Isaac stage run by
# hand), 81-88 (that workspace's motion-planning launch tests) and 91 (its
# Isaac actuator-parity test). Those are the collisions that matter, because
# each has something actuated on the far end. Widening the spread below is
# therefore not free -- 60 + pid % 40 would reach into 81-88 and 91.
#
# Spreading it over the pid makes two concurrent runs of this suite unlikely to
# share a domain rather than guaranteed not to -- one in twenty -- and the cost
# of losing that coin toss is two test runs talking to each other, not a robot
# moving.
#
# The environment is enough here, unlike this project's launch tests: rclpy
# reads the domain when the context is initialised, which is inside the test
# process and always after this module is imported. A launch test has to set it
# through ctest's ENVIRONMENT property because there the DDS layer comes up in
# a process pytest does not own.
#
# LOCALHOST discovery on top, because the domain only separates us from other
# processes; this box shares a network with a real turntable controller that
# has hijacked tests before. It publishes /joint_states rather than /clock or
# /isaac/joint_states, so it happens not to reach these tests today -- which is
# the kind of accident that stops being true silently.
_TEST_DOMAIN_ID = 60 + os.getpid() % 20
os.environ["ROS_DOMAIN_ID"] = str(_TEST_DOMAIN_ID)
os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"

# The launch_testing file is not a plain pytest module; keep pytest from importing it.
collect_ignore_glob = ["*.launch.py"]

FIXTURES_DIR = Path(__file__).resolve().parent / "fixtures"


@pytest.fixture
def fake_serials(monkeypatch):
    """Patch ``serial.serial_for_url`` so USB2FeetechDevice gets a FakeSerial.

    Yields the list of created FakeSerials in construction order, so a test can
    inspect ``trace`` and configure ``servo(id)`` after building a device.
    """
    from ar_gripper.mock import install_fake_serial

    created, _restore = install_fake_serial(monkeypatch)
    return created


@pytest.fixture
def fast_clock(monkeypatch):
    """Replace gripper.py's ``time`` with a deterministic fake clock."""
    from ar_gripper import gripper
    from ar_gripper.mock import FakeTime

    monkeypatch.setattr(gripper, "time", FakeTime())


@pytest.fixture
def make_gripper(fake_serials, fast_clock):
    """Build a real ``Gripper`` on a fresh FakeSerial (fast clock installed).

    Returns ``(gripper, fake_serial)``. ``load_sequence`` seeds the servo's
    present_load reads (for calibration); ``calibrated`` flips the internal flag
    so position moves are accepted without a physical rehome.
    """
    from ar_gripper.feetech import USB2FeetechDevice
    from ar_gripper.gripper import Gripper

    def _make(servo_id=1, name="primary", load_sequence=None, calibrated=False):
        device = USB2FeetechDevice("/dev/fake")
        fake = fake_serials[-1]
        if load_sequence is not None:
            fake.servo(servo_id).load_sequence = list(load_sequence)
        gripper = Gripper(device, name, servo_id)
        if calibrated:
            gripper._calibrated = True
        return gripper, fake

    return _make


@pytest.fixture
def make_standalone(fake_serials, fast_clock, tmp_path):
    """Build an ``ARGripperStandalone`` on a fresh FakeSerial (fast clock).

    A device is built and its servo seeded *before* construction (so calibration
    load sequences / present position are controllable), then injected — the same
    injection path the ROS node uses to share one bus across grippers.

    ``saved_position`` seeds the persisted json (``None`` -> no file, forcing a
    rehome). ``present_position`` seeds the live encoder value. Returns
    ``(standalone, fake_serial, servo_position_path)``.
    """
    import json

    from ar_gripper.feetech import USB2FeetechDevice
    from ar_gripper.standalone import ARGripperStandalone

    def _make(
        servo_id=1,
        name="primary",
        saved_position=150,
        present_position=150,
        load_sequence=None,
        calibrate_on_init=True,
        path=None,
    ):
        device = USB2FeetechDevice("/dev/fake")
        fake = fake_serials[-1]
        servo = fake.servo(servo_id)
        servo.present_position_value = present_position
        if load_sequence is not None:
            servo.load_sequence = list(load_sequence)
        if path is None:
            path = tmp_path / "servo_position.json"
        path = Path(path)
        if saved_position is not None:
            path.write_text(json.dumps({"position": saved_position}))
        standalone = ARGripperStandalone(
            servo_id=servo_id,
            name=name,
            device=device,
            servo_position_path=str(path),
            calibrate_on_init=calibrate_on_init,
        )
        return standalone, fake, path

    return _make
