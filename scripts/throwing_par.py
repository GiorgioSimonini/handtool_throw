#!/usr/bin/env python3
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

# --- global parameters, replaced by yaml --- #
throw_position = [0.3, -0.4, 0.35] # [m] fixed point the object is thrown from
dist_min = 0.05 # [m] a target closer than this to the throwing point has no throwing direction
valve_0 = 0.051
g = 9.81
bound_valve = (0.051, 0.3)
bound_theta = (-np.pi/2, np.pi/2)
IG_valve = 0.052
IG_theta = 0.0
# --- optimization weights and references --- #
valve_ref   = 0.06       # [s]   valve time we trust. Kept well clear of the pole in fun_energy: at 0.052 a 100 us timing jitter moved the landing point by 3.3 cm, at 0.06 by 0.2 cm.
theta_ref   = np.pi/4    # [rad] preferred launch angle
w_valve     = 1.0        # weight on the normalised valve deviation (expensive)
w_theta     = 0.1       # weight on the normalised theta deviation (cheap -> used first)
valve_scale = 0.001      # [s]   normalisation of the valve deviation. NOT the bound range: fun_energy has a pole at valve_0-a3, so the whole useful dynamic range sits within ~1 ms of it. With 0.05 here a valve move looks free and the valve, not theta, ends up doing the aiming.
theta_scale = np.pi/4    # [rad] normalisation of the theta deviation
dist_scale  = 1.0        # [m]   normalisation of the distance constraint
dist_tol    = 1e-3       # [m]   a solution missing by more than this is reported as failed
    
# ----- functions ----- #
def fun_energy(x, x0):
    x = float(x)
    x0 = float(x0)
    # identified parameters
    a1 = 2.0
    a2 = 1.0
    a3 = -0.05092
    b = 20.0
    # the model is a logarithm, it is only defined above the a3 offset
    if (x > x0) and (a2*x + a3 > 0.0) and (a2*x0 + a3 > 0.0):
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
    # - the object always leaves from throw_position: the throw has to cover the
    #   horizontal distance to the target, falling by h on the way - #
    d_xy_target = target - np.array([throw_position[0], throw_position[1], target[2]])
    pos_tool = np.asarray(throw_position, dtype=float)
    dist = np.linalg.norm(d_xy_target)
    h = throw_position[2] - target[2]
    return [dist, h, pos_tool]

def check_target(m_obj, target):
    # - returns an error string, empty if the request can be served - #
    if not np.all(np.isfinite(target)):
        return 'target is not finite'
    if (not np.isfinite(m_obj)) or (m_obj <= 0.0):
        return 'object mass must be positive, got {}'.format(m_obj)
    # the throw leaves from throw_position, so the distance that matters is measured
    # from there: a target on the vertical of the throwing point has no throwing
    # direction and its aiming azimuth would be undefined
    dist_xy = np.linalg.norm(np.asarray(target[0:2]) - np.asarray(throw_position[0:2]))
    if dist_xy <= dist_min:
        return ('target is {:.3f} m from the throwing point in the horizontal plane, '
                'at least {:.3f} m are needed').format(dist_xy, dist_min)
    return ''

def get_landing(m_obj, valve_dt, h, theta):
    v_obj = np.sqrt(2 * fun_energy(valve_dt, valve_0) / m_obj)
    distance = complex(v_obj*cos(theta)/g * ( v_obj * sin(theta) + np.sqrt(v_obj**2 * sin(theta)**2 + 2*g*h)))
    return distance.real

def cost(x):
    # - only decides HOW to hit the target, the distance itself is a constraint - #
    valve_dt = x[0]
    theta = x[1]
    return (w_valve*((valve_dt - valve_ref)/valve_scale)**2
            + w_theta*((theta - theta_ref)/theta_scale)**2)

def constraint_dist(x, m_obj, h, dist_desired):
    # - equality constraint: the throw has to land on the target - #
    return (get_landing(m_obj, x[0], h, x[1]) - dist_desired)/dist_scale

def get_throwing_par(m_obj, target):
    [dist_desired, h, pos_tool] = get_target_info(target)

    lo = np.array([bound_valve[0], bound_theta[0]])
    hi = np.array([bound_valve[1], bound_theta[1]])
    bnds = (tuple(bound_valve), tuple(bound_theta))
    cons = [{'type': 'eq',
             'fun': lambda x: constraint_dist(x, m_obj, h, dist_desired)}]

    # theta is the actuator that does the aiming, so the restarts spread over theta.
    # SLSQP started exactly on a bound (IG_theta = 0) can terminate there after two
    # iterations and still report success, so a single start is not enough.
    starts = [np.array([IG_valve, IG_theta]),
              np.array([valve_ref, theta_ref]),
              np.array([valve_ref, 0.2]),
              np.array([valve_ref, 1.2])]

    feasible = []   # (cost, miss, x) of the starts that actually hit the target
    fallback = None # best effort if none of them does
    for x_start in starts:
        sol = minimize(cost, np.clip(x_start, lo, hi),
                       method='SLSQP', bounds=bnds, constraints=cons)
        if not np.all(np.isfinite(sol.x)):
            continue
        miss = abs(get_landing(m_obj, sol.x[0], h, sol.x[1]) - dist_desired)
        if miss <= dist_tol:
            feasible.append((cost(sol.x), miss, sol.x))
        elif fallback is None or miss < fallback[1]:
            fallback = (cost(sol.x), miss, sol.x)

    if feasible:
        feasible.sort(key=lambda t: t[0])   # among the throws that hit, the cheapest
        obj, miss, x = feasible[0]
        success = True
    elif fallback is not None:
        # the target cannot be reached inside the valve/theta bounds
        obj, miss, x = fallback
        success = False
        rospy.logerr('handtool_server: target out of reach, best miss %.4f m (tolerance %.4f m)',
                     miss, dist_tol)
    else:
        rospy.logerr('handtool_server: optimization returned a non finite solution')
        return [IG_valve, 0.0, pos_tool, np.identity(3), False]

    valve_dt = x[0]
    theta = x[1]
    dist = get_landing(m_obj, valve_dt, h, theta)

    # print solution
    print('Solution')
    print('valve_dt = ' + str(valve_dt) + ' s  (ref ' + str(valve_ref) + ')')
    print('theta    = ' + str(theta) + ' rad  (ref ' + str(theta_ref) + ')')
    print('distance : ' + str(dist) + '  (desired ' + str(dist_desired)
          + ', miss ' + str(dist - dist_desired) + ')')
    print('cost     : ' + str(obj))

    # get R from theta

    # Monaco
    # angle_x = -np.pi/2
    # angle_y = -atan2(target[1], target[0])
    # angle_z = np.pi/2-theta
    # R = np.matmul(R_y(angle_y), R_z(angle_z))
    
    angle_z = atan2(target[1], target[0])
    angle_y = np.pi/2-theta
    R = np.linalg.multi_dot([R_z(angle_z), R_y(angle_y), R_z(np.pi/2),  R_y(-np.pi/2)]) #terna ventosa rispetto MegaPose

    return [valve_dt, theta, pos_tool, R, bool(success)]

# ----- handtool server node ----- #
def callback_throwing_par(req):
    m_obj = req.m_obj
    target = np.array([req.target.x, req.target.y, req.target.z])
    # - reject a request that cannot be served, answer stays False - #
    error = check_target(m_obj, target)
    if error:
        rospy.logerr('handtool_server: rejected request, %s', error)
        return throwing_par_srvResponse(answer = False)
    try:
        [valve_dt, theta, pos_tool, R, success] = get_throwing_par(m_obj, target)
    except (ValueError, ZeroDivisionError, FloatingPointError) as e:
        rospy.logerr('handtool_server: throwing parameters computation failed: %s', e)
        return throwing_par_srvResponse(answer = False)
    if not success:
        return throwing_par_srvResponse(answer = False)
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
    # result_theta is the launch angle over the horizon: the testing node builds the
    # tool orientation from it, the aiming azimuth and its reference orientation
    return throwing_par_srvResponse(result_valve_us = valve_us, result_pose = pose,
                                    result_theta = theta, answer = True)

def handtool_server():
    rospy.init_node('handtool_server')
    # get parameters from yaml
    # with open('../config/handtool_parameters.yaml', 'r') as file:
    #     handtool_params = yaml.safe_load(file)
    global throw_position
    global dist_min
    global valve_0
    global g
    global bound_valve
    global bound_theta
    global IG_valve
    global IG_theta
    global valve_ref
    global theta_ref
    global w_valve
    global w_theta
    global valve_scale
    global theta_scale
    global dist_tol
    # - the module values are used as fallback, so a missing yaml does not kill the node - #
    throw_position = rospy.get_param('optimization/throw_position', throw_position)
    dist_min = rospy.get_param('optimization/dist_min', dist_min)
    valve_0 = rospy.get_param('optimization/valve_0', valve_0)
    g = rospy.get_param('optimization/g', g)
    bound_valve = rospy.get_param('optimization/bound_valve', bound_valve)
    # bound_theta is given in degrees on the parameter server, the optimization works in radians
    bound_theta_deg = rospy.get_param('optimization/bound_theta_deg', None)
    if bound_theta_deg is None:
        rospy.logwarn("handtool_server: 'optimization/bound_theta_deg' not found, using default %s rad", bound_theta)
    else:
        bound_theta = [np.deg2rad(bound_theta_deg[0]), np.deg2rad(bound_theta_deg[1])]
    IG_valve = rospy.get_param('optimization/IG_valve', IG_valve)
    IG_theta = rospy.get_param('optimization/IG_theta', IG_theta)
    # - check the initial guesses lie inside the bounds, SLSQP would clip them silently - #
    IG_valve = min(max(IG_valve, bound_valve[0]), bound_valve[1])
    IG_theta = min(max(IG_theta, bound_theta[0]), bound_theta[1])
    # - cost weights and references - #
    valve_ref = rospy.get_param('optimization/valve_ref', valve_ref)
    theta_ref_deg = rospy.get_param('optimization/theta_ref_deg', None)
    if theta_ref_deg is not None:
        theta_ref = np.deg2rad(theta_ref_deg)
    w_valve = rospy.get_param('optimization/w_valve', w_valve)
    w_theta = rospy.get_param('optimization/w_theta', w_theta)
    valve_scale = rospy.get_param('optimization/valve_scale', valve_scale)
    theta_scale = np.deg2rad(rospy.get_param('optimization/theta_scale_deg',
                                             np.rad2deg(theta_scale)))
    dist_tol = rospy.get_param('optimization/dist_tol', dist_tol)
    # - the references must lie inside the bounds or the cost pulls against them - #
    valve_ref = min(max(valve_ref, bound_valve[0]), bound_valve[1])
    theta_ref = min(max(theta_ref, bound_theta[0]), bound_theta[1])
    rospy.loginfo('handtool_server: valve bounds [%f, %f] s, theta bounds [%f, %f] rad',
                  bound_valve[0], bound_valve[1], bound_theta[0], bound_theta[1])
    rospy.loginfo('handtool_server: cost w_valve %g (ref %.4f s), w_theta %g (ref %.4f rad), '
                  'distance is an equality constraint (tol %g m)',
                  w_valve, valve_ref, w_theta, theta_ref, dist_tol)

    # - advertise the service only once the parameters are loaded - #
    s = rospy.Service('handtool_throw_service', throwing_par_srv, callback_throwing_par)
    rospy.loginfo("handtool_server: 'handtool_throw_service' ready")

    # spin the node
    rospy.spin()

if __name__ == "__main__":
    handtool_server()