#!/usr/bin/env python3
"""Runs the throwing optimizer offline, without ROS and without the robot.

It loads the same 'optimization' block of configs/handtool_params.yaml the node
loads, lets every one of its entries be overridden on the command line, and
prints the solution the service would have answered.

Like the service, --target is measured FROM THE THROWING POINT, not in the robot
base frame: subtract '/testing/throw_pose' position first, or pass it with
--from so that this script does it for you.

  # one throw, target 2.7 m ahead and 1.29 m below the tool
  ./throwing_check.py --mass 0.1 --target 2.7 0.0 -1.29

  # the same target written in the robot base frame
  ./throwing_check.py --mass 0.1 --target 3.0 0.0 -0.94 --from 0.3 -0.4 0.35

  # theta as the only free variable, both branches allowed
  ./throwing_check.py --mass 0.1 --target 2.7 0.0 -1.29 \
      --set free_vars=theta branch=any

  # compare several parameter sets on the same throw
  ./throwing_check.py --mass 0.1 --target 2.7 0.0 -1.29 \
      --case "theta only:free_vars=theta,branch=any" \
      --case "theta low:free_vars=theta,branch=low" \
      --case "both:free_vars=valve+theta,branch=low"

  # table of the throws over a range of distances, all 1.29 m below the tool
  ./throwing_check.py --mass 0.1 --sweep 1.0 6.0 11 --target-z -1.29
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import throwing_model as tm

DEFAULT_YAML = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            '..', 'configs', 'handtool_params.yaml')

# - yaml key -> (attribute, converter). Mirrors load_params() of throwing_par.py - #
YAML_KEYS = {
    'dist_min': ('dist_min', float),
    'valve_0': ('valve_0', float),
    'g': ('g', float),
    'energy_a1': ('energy_a1', float),
    'energy_a2': ('energy_a2', float),
    'energy_a3': ('energy_a3', float),
    'energy_b': ('energy_b', float),
    'bound_valve': ('bound_valve', tuple),
    'bound_theta_deg': ('bound_theta', lambda v: tuple(np.deg2rad(np.asarray(v, float)))),
    'free_vars': ('free_vars', lambda v: [v] if isinstance(v, str) else list(v)),
    'branch': ('branch', str),
    'n_starts': ('n_starts', int),
    'valve_ref': ('valve_ref', float),
    'theta_ref_deg': ('theta_ref', lambda v: float(np.deg2rad(float(v)))),
    'w_valve': ('w_valve', float),
    'w_theta': ('w_theta', float),
    'valve_scale': ('valve_scale', float),
    'theta_scale_deg': ('theta_scale', lambda v: float(np.deg2rad(float(v)))),
    'dist_scale': ('dist_scale', float),
    'dist_tol': ('dist_tol', float),
}


def load_yaml(path):
    """The 'optimization' block of the yaml, empty if the file is not there."""
    if not os.path.isfile(path):
        print('no yaml at %s, using the model defaults' % path)
        return {}
    import yaml
    with open(path, 'r') as fh:
        return (yaml.safe_load(fh) or {}).get('optimization', {}) or {}


def apply_settings(p, settings, where):
    """Applies 'key=value' settings, the keys are the ones of the yaml."""
    for key, value in settings.items():
        if key not in YAML_KEYS:
            raise SystemExit('%s: unknown parameter %r, known ones are:\n  %s'
                             % (where, key, ', '.join(sorted(YAML_KEYS))))
        attr, conv = YAML_KEYS[key]
        setattr(p, attr, conv(value))
    return p


def parse_override(text):
    """'a=1 b=x+y' -> {'a': 1.0, 'b': ['x', 'y']}, lists are written with '+'."""
    if '=' not in text:
        raise SystemExit('overrides are written key=value, got %r' % text)
    key, _, raw = text.partition('=')
    key, raw = key.strip(), raw.strip()
    if '+' in raw:
        return key, [_scalar(v) for v in raw.split('+')]
    return key, _scalar(raw)


def _scalar(raw):
    try:
        return float(raw)
    except ValueError:
        return raw


def build_params(yaml_block, overrides):
    p = tm.ThrowingParams()
    apply_settings(p, {k: v for k, v in yaml_block.items() if k in YAML_KEYS}, 'yaml')
    unknown = [k for k in yaml_block if k not in YAML_KEYS]
    if unknown:
        print('yaml: ignored unknown optimization keys %s' % sorted(unknown))
    apply_settings(p, overrides, 'command line')
    for msg in p.sanitize():
        print('warning: %s' % msg)
    return p


def targets_of(args):
    """Targets to solve, every one of them relative to the throwing point."""
    if args.sweep is not None:
        lo, hi, n = float(args.sweep[0]), float(args.sweep[1]), int(args.sweep[2])
        # the sweep is over the distance from the throwing point, along +x
        return [np.array([d, 0.0, args.target_z]) for d in np.linspace(lo, hi, n)]
    # --from turns a target written in the robot base frame into a relative one
    return [np.asarray(args.target, dtype=float) - np.asarray(args.origin, dtype=float)]


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--yaml', default=DEFAULT_YAML, help='parameter file to start from')
    ap.add_argument('--mass', type=float, default=0.1, help='[kg] object mass')
    ap.add_argument('--target', type=float, nargs=3, default=[2.7, 0.0, -1.29],
                    metavar=('X', 'Y', 'Z'),
                    help='[m] target measured from the throwing point')
    ap.add_argument('--from', dest='origin', type=float, nargs=3, default=[0.0, 0.0, 0.0],
                    metavar=('X', 'Y', 'Z'),
                    help='[m] throwing point, subtracted from --target. Pass the position '
                         'of /testing/throw_pose to write the target in the robot base frame')
    ap.add_argument('--sweep', nargs=3, metavar=('D_MIN', 'D_MAX', 'N'), default=None,
                    help='[m] table over N distances from the throwing point instead')
    ap.add_argument('--target-z', type=float, default=-1.29,
                    help='[m] height of the swept targets, below the throwing point')
    ap.add_argument('--set', nargs='*', default=[], metavar='KEY=VALUE',
                    help="override any yaml parameter, lists are written a+b")
    ap.add_argument('--case', action='append', default=[], metavar='"NAME:K=V,K=V"',
                    help='compare several parameter sets on the same throw')
    args = ap.parse_args()

    yaml_block = load_yaml(args.yaml)
    overrides = dict(parse_override(t) for t in args.set)

    # - the cases share the yaml and the --set overrides, and add their own - #
    cases = []
    for text in args.case:
        name, _, rest = text.partition(':')
        extra = dict(parse_override(t) for t in rest.split(',') if t.strip())
        merged = dict(overrides)
        merged.update(extra)
        cases.append((name.strip(), merged))
    if not cases:
        cases = [('', overrides)]

    targets = targets_of(args)

    for name, settings in cases:
        p = build_params(yaml_block, settings)
        header = 'case %s' % name if name else 'parameters of %s' % os.path.basename(args.yaml)
        print('\n=== %s ===' % header)
        print('free_vars %s | branch %s | valve_ref %.5f s | theta_ref %.1f deg | '
              'w_valve %g scale %g | w_theta %g scale %.1f deg'
              % (p.free_vars, p.branch, p.valve_ref, np.rad2deg(p.theta_ref),
                 p.w_valve, p.valve_scale, p.w_theta, np.rad2deg(p.theta_scale)))
        if args.sweep is not None:
            print('%9s %8s %9s %8s %8s %7s %9s %9s %6s'
                  % ('want[m]', 'got[m]', 'valve[us]', 'theta', 'v[m/s]', 'apex', 'm/100us',
                     'm/deg', 'ok'))
            for target in targets:
                sol = tm.solve(p, args.mass, target)
                sv, st = sol.sensitivity()
                print('%9.3f %8.3f %9d %8.2f %8.2f %7.2f %9.4f %9.4f %6s'
                      % (sol.dist_desired, sol.dist, int(sol.valve_dt * 1e6),
                         np.rad2deg(sol.theta), sol.speed, sol.apex, sv, st,
                         'yes' if sol.success else 'NO'))
        else:
            for target in targets:
                error = tm.check_target(p, args.mass, target)
                if error:
                    print('rejected: %s' % error)
                    continue
                print(tm.solve(p, args.mass, target).report())


if __name__ == '__main__':
    main()
