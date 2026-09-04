# What one gripper's fingers are doing right now, as a level (not an event).
#
# The driver already measures all of this, but until now the only way out was
# the GripperCommand action result: delivered once, at the end of a goal, to
# whoever sent the goal -- in practice a controller manager, which drops it.
# Nothing else could ask "is the gripper holding something?". /joint_states is
# not an answer either: it carries position (and now effort), but not whether
# the fingers stopped early, or which way they were going when they did.

std_msgs/Header header
string          name                     # gripper name, as used in its action/service namespace (e.g. "primary"); its finger joint in /joint_states is "<name>_ar_gripper_body_finger1"
float64         position                 # metres, one finger's travel
float64         effort                   # newtons

# Whether the fingers stopped on something, and which way they were travelling
# when they did. Four values rather than a bool because the direction is
# load-bearing: stopped while OPENING means jammed, not grasped, and a consumer
# that saw only "stalled" would have to treat a failure as a successful grasp.
#
# These are Robotiq's four gOBJ object-detection values, kept by name and by
# number because they are the de-facto ROS 2 vocabulary for this: PickNik's
# ros2_epick_gripper exposes the same concept under the same name. There is no
# standard message for it in control_msgs or sensor_msgs, which is why this
# message exists at all rather than reusing one.
#
# A move still in flight has a direction too, and reporting it is the whole
# reason 4 and 5 exist. They used to read UNKNOWN, which overloaded that value
# with "no command has ever been issued" -- two states that share nothing. The
# distinction matters most exactly where the fingers are NOT stalled: a consumer
# watching a held object cannot otherwise tell "the fingers are running inward
# because what they were pushing on has gone" from "the fingers are on their way
# apart because they were told to let go", and those demand opposite responses.
#
# 0-3 are Robotiq's gOBJ values and keep their numbers; 4 and 5 extend past
# gOBJ, which has no way to say this.
uint8 UNKNOWN          = 0  # no command issued yet, or the gripper is not calibrated
uint8 DETECTED_OPENING = 1  # stopped on contact while opening, before the requested position
uint8 DETECTED_CLOSING = 2  # stopped on contact while closing, before the requested position
uint8 NO_OBJECT        = 3  # reached the requested position: nothing detected, or the object was lost
uint8 OPENING          = 4  # short of a wider requested position, nothing stopping it yet
uint8 CLOSING          = 5  # short of a narrower requested position, nothing stopping it yet
uint8           object_detection_status

bool            moving
bool            calibrated
