#!/usr/bin/env python3
"""Ballistic model and throwing optimizer of the hand tool.

Everything here is PLANAR and relative: the throwing point is the origin, the
object leaves it at a speed set by its launch energy and at an angle theta over
the horizon, and the target is given as a vector from that same origin.

The launch energy comes from ONE switch, ThrowingParams.energy_source:
  'valve'     energy(valve_dt) of the identified valve model, the optimizer moves
              the free_vars among valve time and theta (the original behaviour)
  'measured'  energy measured per object (configs/ball_landing_points.yaml) and
              given with the request, theta is the only unknown and is solved in
              closed form, the valve is always opened for used_valve_us Where the
tool actually is, and which way it has to be turned to aim, is none of this
module's business: the caller places the throw (testing.cpp does it out of
'/testing/throw_pose').

There is no ROS here on purpose: throwing_par.py wraps it in the service node,
throwing_check.py runs exactly the same code offline so a parameter set can be
tried out without the robot.
"""
import numpy as np
from math import log, sin, cos, sqrt, atan2
from scipy.optimize import minimize

VAR_NAMES = ('valve', 'theta')   # order of the optimization vector x
BRANCHES = ('low', 'high', 'any')
ENERGY_SOURCES = ('valve', 'measured')


# ----- parameters ----- #
class ThrowingParams(object):
    """Every knob of the throw, defaults mirror configs/handtool_params.yaml."""

    def __init__(self):
        # --- physics --- #
        self.dist_min = 0.05        # [m] a target closer than this to the throwing point
                                    #     has no throwing direction, its azimuth is undefined
        self.valve_0 = 0.051        # [s] valve time that releases no energy
        self.g = 9.81               # [m/s^2]
        # --- where the launch energy comes from, see the module docstring --- #
        self.energy_source = 'measured'
        self.used_valve_us = 70000  # [us] valve time of every 'measured' throw, the one
                                    #      the energies were measured with
        # --- identified energy model, E(valve_dt) = a1*log_b(a2*t+a3) - (same at valve_0) --- #
        self.energy_a1 = 2.0
        self.energy_a2 = 1.0
        self.energy_a3 = -0.05092
        self.energy_b = 20.0
        # --- search space --- #
        self.bound_valve = (0.051, 0.5)                # [s]
        self.bound_theta = (0.0, np.pi / 2)            # [rad]
        # --- what the optimizer is allowed to move --- #
        # any subset of VAR_NAMES, the variables left out stay frozen at their *_ref
        self.free_vars = ['valve', 'theta']
        # which side of the ballistic apex the solution may sit on:
        #   'low'  flat throw, the shortest time of flight, distance grows with theta
        #   'high' lobbed throw, distance shrinks as theta grows
        #   'any'  whichever of the two the cost likes best
        self.branch = 'low'
        self.n_starts = 5           # multi start grid points per free variable
        # --- cost: how far the solution may sit from the references --- #
        self.valve_ref = 0.06       # [s]   valve time we trust
        self.theta_ref = np.pi / 4  # [rad] preferred launch angle
        self.w_valve = 1.0
        self.w_theta = 0.2
        self.valve_scale = 0.001    # [s]   normalisation of the valve deviation
        self.theta_scale = np.pi / 4  # [rad] normalisation of the theta deviation
        self.dist_scale = 1.0       # [m]   normalisation of the distance constraint
        self.dist_tol = 1e-2        # [m]   a solution missing by more than this failed

    # - keeps the references and the guesses inside the bounds, returns the complaints - #
    def sanitize(self):
        msgs = []
        self.free_vars = [v for v in VAR_NAMES if v in self.free_vars]
        if not self.free_vars:
            msgs.append("free_vars is empty, nothing can be optimized: falling back to ['theta']")
            self.free_vars = ['theta']
        if self.energy_source not in ENERGY_SOURCES:
            msgs.append("energy_source '%s' is not one of %s, using 'valve'"
                        % (self.energy_source, ENERGY_SOURCES))
            self.energy_source = 'valve'
        self.used_valve_us = int(self.used_valve_us)
        if self.branch not in BRANCHES:
            msgs.append("branch '%s' is not one of %s, using 'any'" % (self.branch, BRANCHES))
            self.branch = 'any'
        self.n_starts = max(2, int(self.n_starts))
        self.bound_valve = (float(self.bound_valve[0]), float(self.bound_valve[1]))
        self.bound_theta = (float(self.bound_theta[0]), float(self.bound_theta[1]))
        # the energy model is a logarithm with a pole at valve_0: on the pole the throw
        # has no energy and the gradient is infinite, so the lower bound must clear it
        if self.bound_valve[0] <= self.valve_0:
            msgs.append('bound_valve[0] = %.5f s sits on the energy pole (valve_0 = %.5f s), '
                        'the solver sees an infinite gradient there'
                        % (self.bound_valve[0], self.valve_0))
        for name, bound in (('valve', self.bound_valve), ('theta', self.bound_theta)):
            ref = getattr(self, name + '_ref')
            clipped = min(max(ref, bound[0]), bound[1])
            if clipped != ref:
                msgs.append('%s_ref %.5f is outside its bounds, clipped to %.5f'
                            % (name, ref, clipped))
                setattr(self, name + '_ref', clipped)
        return msgs

    def bounds(self):
        return (self.bound_valve, self.bound_theta)

    def refs(self):
        return np.array([self.valve_ref, self.theta_ref])


# ----- physics ----- #
def energy(p, valve_dt):
    """[J] energy given to the object by a valve opening of valve_dt seconds."""
    x = float(valve_dt)
    x0 = float(p.valve_0)
    a1, a2, a3, b = p.energy_a1, p.energy_a2, p.energy_a3, p.energy_b
    # the model is a logarithm, it is only defined above the a3 offset
    if (x > x0) and (a2 * x + a3 > 0.0) and (a2 * x0 + a3 > 0.0):
        return float(a1 * log(a2 * x + a3, b) - a1 * log(a2 * x0 + a3, b))
    return 0.0


def speed_of_energy(m_obj, e):
    """[m/s] speed of an object of m_obj [kg] carrying e [J]."""
    return sqrt(2.0 * e / m_obj)


def speed(p, m_obj, valve_dt):
    """[m/s] speed the object leaves the tube with, valve model."""
    return speed_of_energy(m_obj, energy(p, valve_dt))


# - the ballistic functions below take the launch speed v [m/s], whatever its source - #
def landing(p, v, h, theta):
    """[m] horizontal distance covered while falling by h.

    The discriminant is clamped at zero so the function stays real and
    continuous for the solver. Clamping means the apex of the trajectory is
    below the target height, i.e. the target is NOT reached: the returned
    value is then the horizontal distance of the apex, the boundary case.
    Use reaches() to tell a real solution from that boundary.
    """
    disc = v * v * sin(theta) ** 2 + 2.0 * p.g * h
    return v * cos(theta) / p.g * (v * sin(theta) + sqrt(max(disc, 0.0)))


def reaches(p, v, h, theta):
    """False when the object never comes down to the height of the target."""
    return (v > 0.0) and (v * v * sin(theta) ** 2 + 2.0 * p.g * h >= 0.0)


def apex_angle(p, v, h):
    """[rad] launch angle of maximum range: the border between the two branches.

    Below it the distance grows with theta (flat throw), above it the distance
    shrinks again (lob). Classic result for a throw from a height h.
    """
    if v <= 0.0:
        return np.pi / 4
    return atan2(v, sqrt(max(v * v + 2.0 * p.g * h, 0.0)))


def apex_height(p, v, theta):
    """[m] how high above the throwing point the object goes."""
    return (v * sin(theta)) ** 2 / (2.0 * p.g)


def flight_time(p, v, h, theta):
    """[s] time between the release and the landing."""
    disc = v * v * sin(theta) ** 2 + 2.0 * p.g * h
    return (v * sin(theta) + sqrt(max(disc, 0.0))) / p.g


def target_info(target):
    """Reduces a target to the plane of the throw.

    target is measured FROM THE THROWING POINT, so the horizontal distance to
    cover is its own norm in xy and the height to fall is minus its z.
    """
    target = np.asarray(target, dtype=float)
    dist = float(np.linalg.norm(target[0:2]))
    h = float(-target[2])
    return [dist, h]


def check_target(p, m_obj, e_obj, target):
    """Error string, empty when the request can be served.

    e_obj [J] is the measured launch energy, only read when energy_source is 'measured'.
    """
    if not np.all(np.isfinite(target)):
        return 'target is not finite'
    if (not np.isfinite(m_obj)) or (m_obj <= 0.0):
        return 'object mass must be positive, got {}'.format(m_obj)
    if (p.energy_source == 'measured') and ((not np.isfinite(e_obj)) or (e_obj <= 0.0)):
        return ("energy_source is 'measured' but the object energy is {} J: give the "
                "measured launch energy of the object").format(e_obj)
    # a target on the vertical of the throwing point has no throwing direction
    # and its aiming azimuth would be undefined
    dist_xy = float(np.linalg.norm(np.asarray(target[0:2], dtype=float)))
    if dist_xy <= p.dist_min:
        return ('target is {:.3f} m from the throwing point in the horizontal plane, '
                'at least {:.3f} m are needed').format(dist_xy, p.dist_min)
    return ''


# ----- optimization ----- #
def cost(p, x):
    """Only decides HOW to hit the target, the distance itself is a constraint."""
    return (p.w_valve * ((x[0] - p.valve_ref) / p.valve_scale) ** 2
            + p.w_theta * ((x[1] - p.theta_ref) / p.theta_scale) ** 2)


def _search_bounds(p):
    """Bounds of the solver: a variable left out of free_vars is pinned to its ref."""
    out = []
    for name, bound in zip(VAR_NAMES, p.bounds()):
        if name in p.free_vars:
            out.append(bound)
        else:
            ref = getattr(p, name + '_ref')
            out.append((ref, ref))
    return tuple(out)


def _starts(p, bnds):
    """Multi start grid, one axis per free variable.

    SLSQP started exactly on a bound can terminate there after two iterations
    and still report success, and both branches of the trajectory are local
    solutions, so a single start is never enough.
    """
    axes = []
    for name, (lo, hi) in zip(VAR_NAMES, bnds):
        if lo == hi:
            axes.append([lo])
            continue
        # the grid stays off the bounds, the reference is added as its own start
        edge = 0.05 * (hi - lo)
        grid = list(np.linspace(lo + edge, hi - edge, p.n_starts))
        grid.append(min(max(getattr(p, name + '_ref'), lo), hi))
        axes.append(sorted(set(np.round(grid, 9))))
    return [np.array([v, t]) for v in axes[0] for t in axes[1]]


class Solution(object):
    """Everything the node and the offline checker need to report a throw."""

    def __init__(self, p, m_obj, target):
        self.p = p
        self.m_obj = float(m_obj)
        self.target = np.asarray(target, dtype=float)   # from the throwing point
        self.dist_desired, self.h = target_info(self.target)
        self.valve_dt = p.valve_ref
        self.theta = p.theta_ref
        self.v = 0.0                # [m/s] launch speed of the accepted throw
        # nan until a solution is accepted: a plain 0.0 reads like a real throw
        self.dist = float('nan')
        self.miss = float('inf')
        self.obj = float('inf')
        self.success = False
        self.reason = 'not solved'
        self.n_feasible = 0
        self.n_starts = 0

    # - derived quantities, all recomputed from the accepted solution - #
    @property
    def speed(self):
        return self.v

    @property
    def energy(self):
        return 0.5 * self.m_obj * self.v ** 2

    @property
    def branch(self):
        apex = apex_angle(self.p, self.v, self.h)
        return 'low' if self.theta <= apex + 1e-9 else 'high'

    @property
    def apex(self):
        return apex_height(self.p, self.v, self.theta)

    @property
    def time_of_flight(self):
        return flight_time(self.p, self.v, self.h, self.theta)

    def sensitivity(self):
        """[m per 100 us, m per deg] how far the landing point moves with a small error.

        This is what makes a solution usable or not on the real tool: the valve
        timing has a jitter of some tens of microseconds and theta is only as
        good as the controller tracking at the release. With a 'measured' energy
        the valve time does not set the speed, its sensitivity is nan.
        """
        d_theta = np.deg2rad(1e-3)
        fd_theta = (landing(self.p, self.v, self.h, self.theta + d_theta)
                    - landing(self.p, self.v, self.h, self.theta - d_theta)) / (2 * d_theta)
        fd_valve = float('nan')
        if self.p.energy_source == 'valve':
            d_valve = 1e-6
            v_plus = speed(self.p, self.m_obj, self.valve_dt + d_valve)
            v_minus = speed(self.p, self.m_obj, self.valve_dt - d_valve)
            fd_valve = (landing(self.p, v_plus, self.h, self.theta)
                        - landing(self.p, v_minus, self.h, self.theta)) / (2 * d_valve)
        return [fd_valve * 1e-4, fd_theta * np.deg2rad(1.0)]

    def report(self):
        s_valve, s_theta = self.sensitivity()
        lines = []
        if self.p.energy_source == 'valve':
            moved = 'free: %s' % ','.join(self.p.free_vars)
        else:
            moved = 'measured energy, free: theta'
        lines.append('throw %s   (%s, branch: %s)'
                     % ('SOLVED' if self.success else 'FAILED', moved, self.p.branch))
        lines.append('  target      : [%.3f, %.3f, %.3f] m from the throwing point, mass %.3f kg'
                     % (self.target[0], self.target[1], self.target[2], self.m_obj))
        lines.append('  distance    : %.4f m of %.4f m wanted (miss %+.4f m, tol %.4f m)'
                     % (self.dist, self.dist_desired, self.dist - self.dist_desired, self.p.dist_tol))
        lines.append('  valve_dt    : %.6f s  (%d us, ref %.6f s)'
                     % (self.valve_dt, int(round(self.valve_dt * 1e6)), self.p.valve_ref))
        lines.append('  theta       : %.3f deg  (ref %.3f deg, %s branch)'
                     % (np.rad2deg(self.theta), np.rad2deg(self.p.theta_ref), self.branch))
        lines.append('  speed       : %.3f m/s (%.3f J), apex %.3f m over the tool, flight %.3f s'
                     % (self.speed, self.energy, self.apex, self.time_of_flight))
        lines.append('  sensitivity : %+.4f m per 100 us of valve, %+.4f m per deg of theta'
                     % (s_valve, s_theta))
        lines.append('  cost        : %.3f   (%d of %d starts hit the target)'
                     % (self.obj, self.n_feasible, self.n_starts))
        if not self.success:
            lines.append('  reason      : %s' % self.reason)
        return '\n'.join(lines)


def solve(p, m_obj, e_obj, target):
    """Finds the throw that lands on the target.

    e_obj [J] is the measured launch energy of the object, only used when
    energy_source is 'measured': this is the one place the switch is read.
    """
    if p.energy_source == 'measured':
        return _solve_measured(p, m_obj, e_obj, target)
    return _solve_valve(p, m_obj, target)


def _solve_measured(p, m_obj, e_obj, target):
    """Theta only, the speed is fixed by the measured energy.

    Closed form of the drag free trajectory through the target (R ahead, h below):
        tan(theta) = (v^2 -+ sqrt(v^4 - g (g R^2 - 2 h v^2))) / (g R)
    minus is the flat throw ('low' branch), plus the lob ('high' branch).
    """
    sol_out = Solution(p, m_obj, target)
    sol_out.valve_dt = p.used_valve_us * 1e-6
    sol_out.v = speed_of_energy(m_obj, e_obj)
    v, g = sol_out.v, p.g
    R, h = sol_out.dist_desired, sol_out.h

    disc = v ** 4 - g * (g * R ** 2 - 2.0 * h * v ** 2)
    if disc < 0.0:
        # beyond the maximum range: the apex angle gets the closest, accepted within dist_tol
        candidates = [apex_angle(p, v, h)]
    elif p.branch == 'low':
        candidates = [atan2(v * v - sqrt(disc), g * R)]
    elif p.branch == 'high':
        candidates = [atan2(v * v + sqrt(disc), g * R)]
    else:
        # the one closest to theta_ref, what the cost of the valve solver prefers too
        candidates = sorted([atan2(v * v - sqrt(disc), g * R), atan2(v * v + sqrt(disc), g * R)],
                            key=lambda t: abs(t - p.theta_ref))
    inside = [t for t in candidates if p.bound_theta[0] <= t <= p.bound_theta[1]]
    if not inside:
        sol_out.reason = ('the launch angles that hit the target (%s deg) are outside '
                          'bound_theta [%.1f, %.1f] deg on the %s branch'
                          % (', '.join('%.2f' % np.rad2deg(t) for t in candidates),
                             np.rad2deg(p.bound_theta[0]), np.rad2deg(p.bound_theta[1]),
                             p.branch))
        return sol_out

    sol_out.theta = float(inside[0])
    sol_out.dist = landing(p, v, h, sol_out.theta)
    sol_out.miss = abs(sol_out.dist - R)
    sol_out.obj = 0.0
    sol_out.n_feasible = len(inside)
    sol_out.n_starts = len(candidates)
    sol_out.success = sol_out.miss <= p.dist_tol
    sol_out.reason = '' if sol_out.success else (
        'target out of reach: %.3f m away with %.3f m/s (%.3f J), the maximum range at this '
        'energy is %.3f m' % (R, v, e_obj, sol_out.dist))
    return sol_out


def _solve_valve(p, m_obj, target):
    """Valve time and/or theta, the cheapest throw that lands on the target."""
    sol_out = Solution(p, m_obj, target)
    dist_desired, h = sol_out.dist_desired, sol_out.h

    bnds = _search_bounds(p)
    starts = _starts(p, bnds)
    sol_out.n_starts = len(starts)

    cons = [{'type': 'eq',
             'fun': lambda x: (landing(p, speed(p, m_obj, x[0]), h, x[1]) - dist_desired) / p.dist_scale}]
    if p.branch == 'low':
        # theta below the angle of maximum range: the flattest of the two throws
        cons.append({'type': 'ineq',
                     'fun': lambda x: apex_angle(p, speed(p, m_obj, x[0]), h) - x[1]})
    elif p.branch == 'high':
        cons.append({'type': 'ineq',
                     'fun': lambda x: x[1] - apex_angle(p, speed(p, m_obj, x[0]), h)})

    feasible = []    # (cost, miss, x) of the starts that actually hit the target
    fallback = None  # best effort if none of them does
    for x_start in starts:
        sol = minimize(lambda x: cost(p, x), x_start,
                       method='SLSQP', bounds=bnds, constraints=cons)
        if not np.all(np.isfinite(sol.x)):
            continue
        # the solver reports success on its own model, the acceptance is ours
        v = speed(p, m_obj, sol.x[0])
        if not reaches(p, v, h, sol.x[1]):
            continue
        miss = abs(landing(p, v, h, sol.x[1]) - dist_desired)
        if not np.isfinite(miss):
            continue
        if p.branch != 'any':
            apex = apex_angle(p, v, h)
            wrong = (sol.x[1] > apex + 1e-6) if p.branch == 'low' else (sol.x[1] < apex - 1e-6)
            if wrong:
                continue
        if miss <= p.dist_tol:
            feasible.append((cost(p, sol.x), miss, sol.x))
        elif fallback is None or miss < fallback[1]:
            fallback = (cost(p, sol.x), miss, sol.x)

    if feasible:
        feasible.sort(key=lambda t: t[0])   # among the throws that hit, the cheapest
        sol_out.obj, sol_out.miss, x = feasible[0]
        sol_out.n_feasible = len(feasible)
        sol_out.success = True
        sol_out.reason = ''
    elif fallback is not None:
        sol_out.obj, sol_out.miss, x = fallback
        sol_out.reason = ('target out of reach with these bounds, best miss %.4f m '
                          '(tolerance %.4f m)' % (sol_out.miss, p.dist_tol))
    else:
        sol_out.reason = ('no start produced a throw that comes down to the target height '
                          'on the %s branch' % p.branch)
        return sol_out

    sol_out.valve_dt = float(x[0])
    sol_out.theta = float(x[1])
    sol_out.v = speed(p, m_obj, sol_out.valve_dt)
    sol_out.dist = landing(p, sol_out.v, h, sol_out.theta)
    return sol_out
