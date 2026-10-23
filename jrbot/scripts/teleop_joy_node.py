#!/usr/bin/env python3
"""
Teleoperation for a small 2‑D differential‑drive robot (e.g. a home vacuum cleaner).

* Listens to /joy messages from a Logitech gamepad.
* Converts the joystick axes into geometry_msgs/Twist commands on /cmd_vel.
* Applies a sine remapping to the angular axis so that the robot turns more
  gently when the stick is near its extremes – this gives a smoother user
  experience and reduces “jerky” turning.

Author:  <your name>
Date:    2026‑07‑22
"""

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import math

# --------------------------------------------------------------
# Configuration – tweak these if your robot requires different limits
# --------------------------------------------------------------

MAX_LINEAR_VEL = 0.50      # m/s  (small vacuum robot)
MAX_ANGULAR_VEL = 1.5      # rad/s

# Logitech F310/FC7500 mapping: left stick vertical -> axis[1]
#                                            horizontal -> axis[0]
# Right stick horizontal for turning? We use left stick horizontal.
JOY_LINEAR_AXIS = 1   # left stick Y (forward/back)
JOY_ANGULAR_AXIS = 0  # left stick X (turn)

# --------------------------------------------------------------
# Helper – sine remapping
# --------------------------------------------------------------

def sin_remap(val: float) -> float:
    """
    Map a joystick value in [-1, 1] to a smoother output using sin.
    This keeps the mapping linear near zero but tapers off as you approach
    the extremes, preventing sudden jerks when turning hard.

    Parameters
    ----------
    val : float
        Joystick axis value (expected between -1 and 1)

    Returns
    -------
    float
        Remapped value in [-1, 1]
    """
    # Ensure we stay inside the domain
    val = max(-1.0, min(1.0, val))
    return math.sin(val * math.pi / 2)   # sin maps -π/2..π/2 -> -1..1

# --------------------------------------------------------------
# Node class
# --------------------------------------------------------------

class TeleopJoyNode:
    def __init__(self):
        rospy.init_node('teleop_joy_node', anonymous=False)

        # Publisher: Twist command
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        # Subscriber: Joy messages from the joystick
        self.sub = rospy.Subscriber(
            'joy',
            Joy,
            self.joy_callback,
            queue_size=1,
            tcp_nodelay=True
        )

        rospy.loginfo("Teleop joy node started. Waiting for joystick input...")

    def joy_callback(self, msg: Joy):
        """
        Called whenever a new Joy message is received.
        """
        # Safety check – if the joystick axes array is shorter than expected,
        # ignore this message.
        if len(msg.axes) <= max(JOY_LINEAR_AXIS, JOY_ANGULAR_AXIS):
            rospy.logwarn_throttle(
                5.0,
                "Joystick message does not contain enough axes."
            )
            return

        # Raw joystick values
        raw_linear = msg.axes[JOY_LINEAR_AXIS]   # Forward/backward
        raw_angular = msg.axes[JOY_ANGULAR_AXIS] # Left/right turn

        # Apply sine remapping to angular axis for smooth turning
        remapped_angular = sin_remap(raw_angular)
        # remapped_angular = raw_angular

        # Scale to max velocities
        linear_vel  = raw_linear * MAX_LINEAR_VEL
        angular_vel = remapped_angular * MAX_ANGULAR_VEL

        # Build Twist message
        twist_msg = Twist()
        twist_msg.linear.x = linear_vel          # Forward/backward
        twist_msg.angular.z = angular_vel        # Yaw rotation

        # Publish command
        self.pub.publish(twist_msg)

    def spin(self):
        rospy.spin()

# --------------------------------------------------------------
# Main entry point
# --------------------------------------------------------------

if __name__ == '__main__':
    try:
        teleop_node = TeleopJoyNode()
        teleop_node.spin()
    except rospy.ROSInterruptException:
        pass
