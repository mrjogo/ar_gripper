"""Pytest fixtures for the hardware-free ar_gripper test suite.

The Feetech bus stand-ins live in ``_fakes`` (importable by the offline
golden-capture step too). This module only wires them into pytest and puts the
package source tree on ``sys.path``.

The package under test lives at ``<repo>/ar_gripper/ar_gripper`` (an ament_cmake
package, so there is no editable install); we prepend that directory to
``sys.path`` here so ``import ar_gripper.*`` resolves to the source tree.
"""

import sys
from pathlib import Path

import pytest

# <repo>/ar_gripper/ar_gripper/test/conftest.py -> parents[1] == <repo>/ar_gripper/ar_gripper
_PKG_PARENT = Path(__file__).resolve().parents[1]
if str(_PKG_PARENT) not in sys.path:
    sys.path.insert(0, str(_PKG_PARENT))

# The launch_testing file is not a plain pytest module; keep pytest from importing it.
collect_ignore_glob = ["*.launch.py"]

FIXTURES_DIR = Path(__file__).resolve().parent / "fixtures"


@pytest.fixture
def fake_serials(monkeypatch):
    """Patch ``serial.serial_for_url`` so USB2FeetechDevice gets a FakeSerial.

    Yields the list of created FakeSerials in construction order, so a test can
    inspect ``trace`` and configure ``servo(id)`` after building a device.
    """
    from _fakes import install_fake_serial

    created, _restore = install_fake_serial(monkeypatch)
    return created


@pytest.fixture
def fast_clock(monkeypatch):
    """Replace gripper.py's ``time`` with a deterministic fake clock."""
    from _fakes import FakeTime

    from ar_gripper import gripper

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
