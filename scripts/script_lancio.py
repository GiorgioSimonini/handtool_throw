#!/usr/bin/env python3

import numpy as np

import rospy
# from std_msgs.msg import String
from handtool_throw.srv import throwing_par_srv, throwing_par_srvRequest
from geometry_msgs.msg import Point
import tf

def set_throw_param():
        
        param = rospy.ServiceProxy('handtool_throw_service', throwing_par_srv)

        f = throwing_par_srvRequest()

        # Get user input
        print("insert object weight and target x, y, z positions (m_obj x y z):")

        # Read input values
        m_obj = float(input("Object weight (m_obj): "))
        x, y, z = map(float, input("Target x, y, z positions: ").split())

        # Set service request parameters
        f.m_obj = m_obj

        target = Point()
        target.x = x
        target.y = y
        target.z = z

        f.target = target

        resp1 = param(f)

        print(resp1)

        t_0V = np.array([resp1.result_pose.position.x, resp1.result_pose.position.y, resp1.result_pose.position.z])
        q_0V = np.array([resp1.result_pose.orientation.x, resp1.result_pose.orientation.y, resp1.result_pose.orientation.z, resp1.result_pose.orientation.w])

        print("Ventosa pose:", t_0V[0], t_0V[1], t_0V[2], q_0V[0], q_0V[1], q_0V[2], q_0V[3])

        # Create homogeneous transformation matrix from translation and quaternion
        T_0V = tf.transformations.quaternion_matrix(q_0V)
        T_0V[0:3, 3] = t_0V

        # Get the transformation between the tool_extremity and the hand
        while True:
            try:
                # Fixed: corrected frame order (from -> to)
                (t_VE, q_VE) = listener.lookupTransform("tool_extremity", "right_hand_ee_link", rospy.Time(0))
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

        # Create homogeneous transformation matrix from TF lookup
        T_VE = tf.transformations.quaternion_matrix(q_VE)
        T_VE[0:3, 3] = t_VE

        translation_VE = T_VE[0:3, 3]
        quaternion_VE = tf.transformations.quaternion_from_matrix(T_VE)
        
        print("Tool-Hand pose:", translation_VE[0], translation_VE[1], translation_VE[2], quaternion_VE[0], quaternion_VE[1], quaternion_VE[2], quaternion_VE[3])

        # Multiply the transformation matrices
        T_result = np.dot(T_0V, T_VE)
        
        translation_result = T_result[0:3, 3]
        quaternion_result = tf.transformations.quaternion_from_matrix(T_result)
        
        print("Final pose:", translation_result[0], translation_result[1], translation_result[2], quaternion_result[0], quaternion_result[1], quaternion_result[2], quaternion_result[3])

if __name__ == "__main__":

    # Initialize ROS node first
    rospy.init_node('throwing_client', anonymous=True)
    
    # Create TF listener
    listener = tf.TransformListener()
    
    # Give TF listener time to receive transforms
    rospy.sleep(1.0)

    rospy.wait_for_service('handtool_throw_service')

    try:
      
      set_throw_param()

    except rospy.ServiceException as e:

        print("Service call failed: %s"%e)