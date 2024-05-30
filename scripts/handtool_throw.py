#!/usr/bin/env python3
# license removed for brevity
import numpy as np
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation as rot

import rospy
# from std_msgs.msg import String
from handtool_throw.srv import MyService, MyServiceResponse
from std_msgs.msg import UInt32
from geometry_msgs.msg import Pose, Point, Quaternion

# ----- functions ----- #
def fun_energy(x, x0):
    if x > x0:
        a1 = 2
        a2 = 1.0
        a3 = -0.05092
        b = 20
        y = a1*np.log(a2*x+a3, b) - a1*np.log(a2*x0+a3, b)
    else:
        y = 0
    return y

def R_x(angle):
    R = np.array(((1, 0, 0), (0, np.cos(angle), -np.sin(angle)), (0, np.sin(angle), np.cos(angle))))
    return R

def R_y(angle):
    R = np.array(((np.cos(angle), 0, np.sin(angle)), (0, 1, 0), (-np.sin(angle), 0, np.cos(angle))))
    return R

def R_z(angle):
    R = np.array(((np.cos(angle), -np.sin(angle), 0), (np.sin(angle), np.cos(angle), 0), (0, 0, 1)))
    return R

def get_target_info(target):
    dist_base = 0.5 # plant distance from base
    h_base = 0.4	# tool height
    
    d_xy_target = target - np.array([0, 0, target[2]])
    pos_tool = d_xy_target/np.linalg.norm(d_xy_target)*dist_base
    pos_tool[2] = h_base
    
    dist = np.linalg.norm(d_xy_target) - dist_base
    
    h = target[2]-h_base
    
    return [dist, h, pos_tool]

def get_landing(m_obj, valve_dt, h, theta):
    valve_0 = 0.051
    g = 9.81
    v_obj = np.sqrt(2 * fun_energy(valve_dt, valve_0) / m_obj)
    distance = complex(v_obj*np.cos(theta)/g * ( v_obj * np.sin(theta) + np.sqrt(v_obj**2 * np.sin(theta)**2 + 2*g*h)))
    return distance.real

def objective(x, par):
    _m_obj = par[0]
    target = par[1]
    [dist_desired, h, pos_tool]  = get_target_info(target)
    valve_dt = x[0]
    theta = x[1]
    dist = get_landing(_m_obj, valve_dt, h, theta)
    obj = (dist_desired-dist)**2
    return obj

# def constraint(x):
#     return x[0] - 0.051

def get_throwing_par(_m_obj, target):
    [target_dist, h, pos_tool] = get_target_info(target)
    # - solve optimization - #
    # - parameters - #
    par = (_m_obj, target) # tuple of parameters to pass
    # initial guesses
    x0 = np.zeros(2)
    x0[0] = 0.052
    x0[1] = 0
    
    # show initial objective
    print('Initial SSE Objective: ' + str(objective(x0, par)))
    
    # optimize
    b_valve = (0.051, 0.3)
    b_theta = (-3.1415/2, 3.1415/2)
    bnds = (b_valve, b_theta)
    # con1 = {'type': 'ineq', 'fun': constraint}
    # cons = ([con1])
    # solution = minimize(objective, x0, args=(par,), method='SLSQP', bounds=bnds, constraints=cons)
    solution = minimize(lambda x : objective(x, par), x0,  method='SLSQP', bounds=bnds)

    x = solution.x
    valve_dt = x[0]
    theta = x[1]
    # show final objective
    print('Final SSE Objective: ' + str(objective(x, par)))
    
    # print solution
    print('Solution')
    print('x1 = ' + str(x[0]))
    print('x2 = ' + str(x[1]))
    dist = get_landing(_m_obj, valve_dt, h, theta)
    print('distance: ' + str(dist))

    # get R from theta
    angle_z = np.atan2(target[1], target[0])
    angle_y = theta
    R = R_z(angle_z) * R_y(-3.1415-angle_y)
    
    return [valve_dt, pos_tool, R]

# ----- handtool server node ----- #
def callback_handtool_throw(req):
    # target = np.array([2.0, 0.0, 0.0])
    m_obj = req.m_obj
    target = np.array([req.target.x, req.target.y, req.target.z])
    [valve_dt, pos_tool, R] = get_throwing_par(m_obj, target)
    valve_us = UInt32(valve_dt*1e6)
    pose = Pose()
    r = rot.from_matrix(R)
    pose.position.x = pos_tool[0]
    pose.position.y = pos_tool[1]
    pose.position.z = pos_tool[2]
    pose.quaternion = Quaternion(r.as_quat)
    return MyServiceResponse(result_valve_us = valve_us, result_pose = pose)

def handtool_server():
    rospy.init_node('handtool_server')
    s = rospy.Service('handtool_throw', MyService, callback_handtool_throw)
    rospy.spin()

if __name__ == "__main__":
    handtool_server()