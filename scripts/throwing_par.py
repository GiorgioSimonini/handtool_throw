#!/usr/bin/env python
# license removed for brevity
import numpy as np
from math import log, sin, cos, atan2
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation as rot
import yaml

import rospy
# from std_msgs.msg import String
from handtool_throw.srv import throwing_par_srv, throwing_par_srvResponse
from std_msgs.msg import UInt32
from geometry_msgs.msg import Pose, Point, Quaternion

# --- global parameters --- #
dist_base = 0.5 # plant distance from base
h_base = 0.4	# tool height
valve_0 = 0.051
g = 9.81
bound_valve = (0.051, 0.3)
bound_theta = (-3.1415/2, 3.1415/2)
IG_valve = 0.052
IG_theta = 0.0
    
# ----- functions ----- #
def fun_energy(x, x0):
    x = float(x)
    x0 = float(x0)
    if x > x0:
        # identified parameters
        a1 = 2.0
        a2 = 1.0
        a3 = -0.05092
        b = 20.0
        y = float(a1*log(a2*x+a3, b) - a1*log(a2*x0+a3, b))
    else:
        y = 0.0
    return y

def R_x(angle):
    R = np.matrix(((1, 0, 0), (0, cos(angle), -sin(angle)), (0, sin(angle), cos(angle))))
    return R

def R_y(angle):
    R = np.matrix(((cos(angle), 0, sin(angle)), (0, 1, 0), (-sin(angle), 0, cos(angle))))
    return R

def R_z(angle):
    R = np.matrix(((cos(angle), -sin(angle), 0), (sin(angle), cos(angle), 0), (0, 0, 1)))
    return R

def get_target_info(target):
    d_xy_target = target - np.array([0, 0, target[2]])
    pos_tool = d_xy_target/np.linalg.norm(d_xy_target)*dist_base
    pos_tool[2] = h_base
    
    dist = np.linalg.norm(d_xy_target) - dist_base
    
    h = h_base - target[2]
    return [dist, h, pos_tool]

def get_landing(m_obj, valve_dt, h, theta):
    v_obj = np.sqrt(2 * fun_energy(valve_dt, valve_0) / m_obj)
    distance = complex(v_obj*cos(theta)/g * ( v_obj * sin(theta) + np.sqrt(v_obj**2 * sin(theta)**2 + 2*g*h)))
    return distance.real

def objective(x, par):
    m_obj = par[0]
    target = par[1]
    [dist_desired, h, pos_tool]  = get_target_info(target)
    valve_dt = x[0]
    theta = x[1]
    dist = get_landing(m_obj, valve_dt, h, theta)
    obj = (dist_desired-dist)**2
    return obj

# def constraint(x):
#     return x[0] - 0.051

def get_throwing_par(m_obj, target):
    [target_dist, h, pos_tool] = get_target_info(target)
    # - solve optimization - #
    # - parameters - #
    par = (m_obj, target) # tuple of parameters to pass
    # initial guesses
    x0 = np.zeros(2)
    x0[0] = IG_valve
    x0[1] = IG_theta
    
    # show initial objective
    print('Initial SSE Objective: ' + str(objective(x0, par)))
    
    # optimize
    bnds = (bound_valve, bound_theta)
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
    dist = get_landing(m_obj, valve_dt, h, theta)
    print('distance: ' + str(dist))

    # get R from theta
    angle_z = atan2(target[1], target[0])
    angle_y = -3.1415-theta
    R = np.matmul(R_z(angle_z), R_y(angle_y))
    
    return [valve_dt, pos_tool, R]

# ----- handtool server node ----- #
def callback_throwing_par(req):
    m_obj = req.m_obj
    target = np.array([req.target.x, req.target.y, req.target.z])
    [valve_dt, pos_tool, R] = get_throwing_par(m_obj, target)
    valve_us = int(valve_dt*1e6)
    r = rot.from_matrix(R)
    print("R: ")
    print(R)
    quat = r.as_quat()
    pose = Pose()
    pose.position.x = pos_tool[0]
    pose.position.y = pos_tool[1]
    pose.position.z = pos_tool[2]
    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]
    return throwing_par_srvResponse(result_valve_us = valve_us, result_pose = pose)

def handtool_server():
    rospy.init_node('handtool_server')
    s = rospy.Service('throwing_par_srv', throwing_par_srv, callback_throwing_par)
    # get parameters from yaml
    # with open('../config/handtool_parameters.yaml', 'r') as file:
    #     handtool_params = yaml.safe_load(file)
    global dist_base
    global h_base
    global valve_0
    global g
    global bound_valve
    global bound_theta
    global IG_valve
    global IG_theta
    # dist_base = handtool_params['optimization']['dist_base']
    # h_base = handtool_params['optimization']['h_base']
    # valve_0 = handtool_params['optimization']['valve_0']
    # g = handtool_params['optimization']['g']
    # bound_valve = handtool_params['optimization']['bound_valve']
    # bound_theta = handtool_params['optimization']['bound_theta']
    # IG_valve = handtool_params['optimization']['IG_valve']
    # IG_theta = handtool_params['optimization']['IG_theta']
    dist_base = rospy.get_param('optimization/dist_base')
    h_base = rospy.get_param('optimization/h_base')
    valve_0 = rospy.get_param('optimization/valve_0')
    g = rospy.get_param('optimization/g')
    bound_valve = rospy.get_param('optimization/bound_valve')
    bound_theta = rospy.get_param('optimization/bound_theta')
    IG_valve = rospy.get_param('optimization/IG_valve')
    IG_theta = rospy.get_param('optimization/IG_theta')
    
    # spin the node
    rospy.spin()

if __name__ == "__main__":
    handtool_server()