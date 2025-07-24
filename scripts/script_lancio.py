#!/usr/bin/env python3

import numpy as np

import rospy
from handtool_throw.srv import throwing_par_srv, throwing_par_srvRequest
from geometry_msgs.msg import Point
import tf

def set_throw_param(listener):
    param = rospy.ServiceProxy('handtool_throw_service', throwing_par_srv)
    f = throwing_par_srvRequest()

    print("insert object weight and target x, y, z positions (m_obj x y z):")

    m_obj = float(input("Object weight (m_obj): "))
    x, y, z = map(float, input("Target x, y, z positions: ").split())

    f.m_obj = m_obj
    f.target = Point(x=x, y=y, z=z)

    resp1 = param(f)

    t_0V = np.array([resp1.result_pose.position.x, resp1.result_pose.position.y, resp1.result_pose.position.z])
    q_0V = np.array([resp1.result_pose.orientation.x, resp1.result_pose.orientation.y, resp1.result_pose.orientation.z, resp1.result_pose.orientation.w])

    T_0V = tf.transformations.quaternion_matrix(q_0V)
    T_0V[0:3, 3] = t_0V

    while True:
        try:
            (t_VE, q_VE) = listener.lookupTransform("tool_extremity", "right_hand_ee_link", rospy.Time(0))
            break
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

    T_VE = tf.transformations.quaternion_matrix(q_VE)
    T_VE[0:3, 3] = t_VE

    T_result = np.dot(T_0V, T_VE)

    translation_result = T_result[0:3, 3]
    quaternion_result = tf.transformations.quaternion_from_matrix(T_result)

    print("Final pose:", translation_result[0], translation_result[1], translation_result[2],
          quaternion_result[0], quaternion_result[1], quaternion_result[2], quaternion_result[3])

    return translation_result, quaternion_result


if __name__ == "__main__":

    # Initialize ROS node first
    rospy.init_node('throwing_client', anonymous=True)
    
    # Create TF listener
    listener = tf.TransformListener()
    broadcaster = tf.TransformBroadcaster()

    # Give TF listener time to receive transforms
    rospy.sleep(1.0)

    rospy.wait_for_service('handtool_throw_service')

    try:
        translation_result, quaternion_result = set_throw_param(listener)

        rate = rospy.Rate(10)  # 10 Hz

        print("Posa desiderata pubblicata su TF")
        while not rospy.is_shutdown():
            broadcaster.sendTransform(
                translation_result,
                quaternion_result,
                rospy.Time.now(),
                "robot_arm_link0",   # child frame
                "config"             # parent frame
            )
            rate.sleep()

    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)
