#!/usr/bin/env python3
# license removed for brevity
"""handtool_throw service node: answers with the parameters of one throw.

The request carries the target measured FROM THE THROWING POINT, the answer is
the valve time and the launch angle. The node never sees the throwing pose: the
caller owns it, subtracts it before asking and builds the tool orientation out
of the launch angle afterwards.

The physics and the optimizer live in throwing_model.py, this file only maps the
parameter server onto them and serves 'handtool_throw_service'. Use
throwing_check.py to try a parameter set out without the robot.
"""
import os
import sys

import numpy as np

import rospy
from handtool_throw.srv import throwing_par_srv, throwing_par_srvResponse

# catkin runs this file through a relay that keeps __file__ on the source, so the
# model sitting next to it is found both from the devel and the install space
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import throwing_model as tm

# - filled by load_params(), the defaults of the model are used until then - #
params = tm.ThrowingParams()


# ----- parameters ----- #
def load_params():
    """Reads the 'optimization' namespace, the model defaults are the fallback."""
    p = tm.ThrowingParams()
    p.dist_min = rospy.get_param('optimization/dist_min', p.dist_min)
    p.valve_0 = rospy.get_param('optimization/valve_0', p.valve_0)
    p.g = rospy.get_param('optimization/g', p.g)
    # - identified energy model - #
    p.energy_a1 = rospy.get_param('optimization/energy_a1', p.energy_a1)
    p.energy_a2 = rospy.get_param('optimization/energy_a2', p.energy_a2)
    p.energy_a3 = rospy.get_param('optimization/energy_a3', p.energy_a3)
    p.energy_b = rospy.get_param('optimization/energy_b', p.energy_b)
    # - search space, theta is given in degrees on the parameter server - #
    p.bound_valve = rospy.get_param('optimization/bound_valve', p.bound_valve)
    bound_theta_deg = rospy.get_param('optimization/bound_theta_deg', None)
    if bound_theta_deg is None:
        rospy.logwarn("handtool_server: 'optimization/bound_theta_deg' not found, "
                      "using default %s rad", p.bound_theta)
    else:
        p.bound_theta = (np.deg2rad(bound_theta_deg[0]), np.deg2rad(bound_theta_deg[1]))
    # - what the optimizer may move, and on which side of the apex - #
    p.free_vars = rospy.get_param('optimization/free_vars', p.free_vars)
    p.branch = rospy.get_param('optimization/branch', p.branch)
    p.n_starts = rospy.get_param('optimization/n_starts', p.n_starts)
    # - cost weights and references - #
    p.valve_ref = rospy.get_param('optimization/valve_ref', p.valve_ref)
    theta_ref_deg = rospy.get_param('optimization/theta_ref_deg', None)
    if theta_ref_deg is not None:
        p.theta_ref = np.deg2rad(theta_ref_deg)
    p.w_valve = rospy.get_param('optimization/w_valve', p.w_valve)
    p.w_theta = rospy.get_param('optimization/w_theta', p.w_theta)
    p.valve_scale = rospy.get_param('optimization/valve_scale', p.valve_scale)
    p.theta_scale = np.deg2rad(rospy.get_param('optimization/theta_scale_deg',
                                               np.rad2deg(p.theta_scale)))
    p.dist_scale = rospy.get_param('optimization/dist_scale', p.dist_scale)
    p.dist_tol = rospy.get_param('optimization/dist_tol', p.dist_tol)

    for msg in p.sanitize():
        rospy.logwarn('handtool_server: %s', msg)
    return p


# ----- service callback ----- #
def callback_throwing_par(req):
    m_obj = req.m_obj
    # the request is already relative to the throwing point, see throwing_par_srv.srv
    target = np.array([req.target.x, req.target.y, req.target.z])
    # - reject a request that cannot be served, answer stays False - #
    error = tm.check_target(params, m_obj, target)
    if error:
        rospy.logerr('handtool_server: rejected request, %s', error)
        return throwing_par_srvResponse(answer=False)
    try:
        sol = tm.solve(params, m_obj, target)
    except (ValueError, ZeroDivisionError, FloatingPointError) as e:
        rospy.logerr('handtool_server: throwing parameters computation failed: %s', e)
        return throwing_par_srvResponse(answer=False)

    # loginfo and not print: a python stdout redirected by roslaunch is buffered
    rospy.loginfo('handtool_server:\n%s', sol.report())
    if not sol.success:
        rospy.logerr('handtool_server: %s', sol.reason)
        return throwing_par_srvResponse(answer=False)

    # result_theta is the launch angle over the horizon: the testing node builds the
    # tool orientation from it, the aiming azimuth and its reference orientation
    return throwing_par_srvResponse(result_valve_us=int(sol.valve_dt * 1e6),
                                    result_theta=sol.theta,
                                    answer=True)


# ----- handtool server node ----- #
def handtool_server():
    global params
    rospy.init_node('handtool_server')
    params = load_params()

    rospy.loginfo('handtool_server: valve bounds [%f, %f] s, theta bounds [%f, %f] deg',
                  params.bound_valve[0], params.bound_valve[1],
                  np.rad2deg(params.bound_theta[0]), np.rad2deg(params.bound_theta[1]))
    rospy.loginfo("handtool_server: free variables %s, branch '%s', %d start grid",
                  params.free_vars, params.branch, params.n_starts)
    rospy.loginfo('handtool_server: cost w_valve %g (ref %.4f s), w_theta %g (ref %.2f deg), '
                  'distance is an equality constraint (tol %g m)',
                  params.w_valve, params.valve_ref, params.w_theta,
                  np.rad2deg(params.theta_ref), params.dist_tol)

    # - advertise the service only once the parameters are loaded - #
    rospy.Service('handtool_throw_service', throwing_par_srv, callback_throwing_par)
    rospy.loginfo("handtool_server: 'handtool_throw_service' ready")

    # spin the node
    rospy.spin()


if __name__ == "__main__":
    handtool_server()
