"""Read numbers the rest of the system has to agree on out of the description.

The description is the only place several independent subsystems can agree
about the mechanism: the driver paces its commands from it, Gazebo and MoveIt
build their limits from it, and the Isaac stage is imported from a URDF
expanded from it. Anything that copies one of those numbers into code becomes
the place that goes stale, because nothing compares the copies.

Deliberately free of ROS imports, and free of anything outside the standard
library. Two of the consumers cannot have ROS available: ``mock.py``'s
ROS-free wheel is one, and the more demanding one is Isaac's own interpreter,
where ``barbot_isaac``'s stage script loads this file directly by path (it must
not source a ROS overlay -- see that script for why) so that its staleness
check and the driver are reading the description through the same code rather
than through two parsers that can drift apart. The one ROS-shaped thing here,
resolving the installed package's share directory, is imported inside the
branch that needs it so importing this module never requires it.
"""

from pathlib import Path
from xml.etree import ElementTree

# The description this package owns and every consumer of the finger joint
# shares. The joint is declared with a tf_prefix, so it is matched by the
# unprefixed suffix.
DESCRIPTION_RELPATH = Path("urdf") / "ar_gripper_macro.xacro"
FINGER_JOINT_SUFFIX = "ar_gripper_body_finger1"


def default_description_path():
    """The installed ``ar_gripper_macro.xacro``. Needs ROS; callers with a path do not."""
    from ament_index_python.packages import get_package_share_directory

    return Path(get_package_share_directory("ar_gripper")) / DESCRIPTION_RELPATH


def finger_velocity_limit_mps(description_path=None):
    """Read the finger joint's ``<limit velocity="...">`` out of the description.

    The gripper driver must not command targets faster than the joint's own
    velocity limit, because that same limit is what the simulator clamps the
    simulated joint to -- publish faster and the target simply runs ahead of a
    joint that then catches up on its own schedule. That makes the limit a
    number two subsystems have to agree on, and the description is the one that
    already exists: copying it into a Python constant would make that copy the
    first thing to go stale, since nothing would notice the disagreement.

    The macro is read as plain XML rather than expanded through xacro: the
    value is deliberately a literal there (see the comment above the joints),
    expansion would pull in a build-time tool at node startup, and a literal
    that stops being a literal should fail loudly here rather than be silently
    re-derived. A non-numeric value therefore raises.
    """
    if description_path is None:
        description_path = default_description_path()
    root = ElementTree.parse(description_path).getroot()
    for joint in root.iter("joint"):
        name = joint.get("name") or ""
        if not name.endswith(FINGER_JOINT_SUFFIX):
            continue
        limit = joint.find("limit")
        raw = limit.get("velocity") if limit is not None else None
        if raw is None:
            raise ValueError(
                f"{description_path}: joint {name!r} has no <limit velocity=...>"
            )
        try:
            return float(raw)
        except ValueError as exc:
            raise ValueError(
                f"{description_path}: joint {name!r} has a non-literal velocity "
                f"limit {raw!r}. This file is read as plain XML; either keep the "
                "limit a literal or teach this reader to expand xacro."
            ) from exc
    raise ValueError(
        f"{description_path}: no joint named *{FINGER_JOINT_SUFFIX} to take a "
        "velocity limit from"
    )
