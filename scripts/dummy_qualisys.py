#!/usr/bin/env python3
# license removed for brevity
import numpy as np
from scipy.spatial.transform import Rotation as rot

import rospy
from geometry_msgs.msg import PoseStamped

# --- global parameters --- #
rate_hz = 100.0                                 # [Hz] qualisys publishes at 100 Hz
frame_id = 'mocap'
franka_base_topic = '/qualisys/mpc_franka/pose'
target_topic = '/qualisys/box_target/pose'
franka_base_position = [0.0, 0.0, 0.0]          # [m] robot base in the mocap frame
franka_base_rpy_deg = [0.0, 0.0, 0.0]           # [deg]
target_position = [1.5, 0.0, 0.0]               # [m] target box in the mocap frame
throw_position = [0.3, -0.4, 0.35]              # [m] mirrors testing/throw_pose position
dist_min = 0.05                                 # [m] mirrors optimization/dist_min


# ----- functions ----- #
def get_pose_msg(position, quat, frame):
    msg = PoseStamped()
    msg.header.frame_id = frame
    msg.pose.position.x = position[0]
    msg.pose.position.y = position[1]
    msg.pose.position.z = position[2]
    msg.pose.orientation.x = quat[0]
    msg.pose.orientation.y = quat[1]
    msg.pose.orientation.z = quat[2]
    msg.pose.orientation.w = quat[3]
    return msg


def get_target_in_base(p_target, p_base, R_base):
    # - same transform as targetCallback in testing.cpp - #
    return R_base.T.dot(np.asarray(p_target) - np.asarray(p_base))


# ----- dummy qualisys node ----- #
def dummy_qualisys():
    rospy.init_node('dummy_qualisys')
    # - the module values are used as fallback, so a missing yaml does not kill the node - #
    global rate_hz
    global frame_id
    global franka_base_topic
    global target_topic
    global franka_base_position
    global franka_base_rpy_deg
    global target_position
    rate_hz = rospy.get_param('dummy_qualisys/rate', rate_hz)
    frame_id = rospy.get_param('dummy_qualisys/frame_id', frame_id)
    franka_base_topic = rospy.get_param('dummy_qualisys/franka_base_topic', franka_base_topic)
    target_topic = rospy.get_param('dummy_qualisys/target_topic', target_topic)
    franka_base_position = rospy.get_param('dummy_qualisys/franka_base/position', franka_base_position)
    franka_base_rpy_deg = rospy.get_param('dummy_qualisys/franka_base/rpy_deg', franka_base_rpy_deg)
    target_position = rospy.get_param('dummy_qualisys/target/position', target_position)
    # the throwing point is owned by the testing node, read here only to warn early
    throw_position_par = rospy.get_param('testing/throw_pose/position', throw_position)
    dist_min_par = rospy.get_param('optimization/dist_min', dist_min)

    # - the poses are constant, build the messages once - #
    R_base = rot.from_euler('xyz', np.deg2rad(franka_base_rpy_deg)).as_matrix()
    quat_base = rot.from_matrix(R_base).as_quat()
    msg_franka_base = get_pose_msg(franka_base_position, quat_base, frame_id)
    msg_target = get_pose_msg(target_position, [0.0, 0.0, 0.0, 1.0], frame_id)

    # - the service receives the target in the robot base frame, show it - #
    target_base = get_target_in_base(target_position, franka_base_position, R_base)
    rospy.loginfo('dummy_qualisys: franka base at [%.3f, %.3f, %.3f] m, rpy [%.1f, %.1f, %.1f] deg (%s frame)',
                  franka_base_position[0], franka_base_position[1], franka_base_position[2],
                  franka_base_rpy_deg[0], franka_base_rpy_deg[1], franka_base_rpy_deg[2], frame_id)
    rospy.loginfo('dummy_qualisys: target at [%.3f, %.3f, %.3f] m in the %s frame',
                  target_position[0], target_position[1], target_position[2], frame_id)
    rospy.loginfo('dummy_qualisys: target at [%.3f, %.3f, %.3f] m in the franka base frame',
                  target_base[0], target_base[1], target_base[2])

    # - the optimization rejects a target too close to the throwing point, warn instead
    #   of failing later. The distance is measured from there, not from the base - #
    dist_xy = np.linalg.norm(target_base[0:2] - np.asarray(throw_position_par[0:2]))
    if dist_xy <= dist_min_par:
        rospy.logwarn('dummy_qualisys: target is %.3f m from the throwing point in the horizontal '
                      'plane, less than dist_min (%.3f m), the throwing parameters service will '
                      'reject it', dist_xy, dist_min_par)

    pub_franka_base = rospy.Publisher(franka_base_topic, PoseStamped, queue_size=1)
    pub_target = rospy.Publisher(target_topic, PoseStamped, queue_size=1)
    rospy.loginfo("dummy_qualisys: publishing '%s' and '%s' at %.1f Hz",
                  franka_base_topic, target_topic, rate_hz)

    # - the throwing loop resets its pose flags at every cycle, so keep publishing (no latch) - #
    rate = rospy.Rate(rate_hz)
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        # the base pose goes first, targetCallback discards a target received before it
        msg_franka_base.header.stamp = now
        pub_franka_base.publish(msg_franka_base)
        msg_target.header.stamp = now
        pub_target.publish(msg_target)
        rate.sleep()


if __name__ == "__main__":
    try:
        dummy_qualisys()
    except rospy.ROSInterruptException:
        pass
