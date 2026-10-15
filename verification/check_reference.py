"""Run from repository root: python verification/check_reference.py"""
from reference_simulation import parameters, gains, attitude, flight
import numpy as np
from scipy.linalg import expm
p = parameters()
kp, kd, b = gains(p)
A = np.array([[0, 1], [-b*kp, -b*kd]])
assert np.all(np.real(np.linalg.eigvals(A)) < 0), 'PD poles must be stable'
t, y, delta = attitude(p, 'PD', linear=True)
exact = np.array([expm(A*ti) @ [p['theta0'], p['omega0']] for ti in t])
assert np.max(np.abs(exact-y)) < 1e-7, 'Linear ODE differs from matrix exponential'
assert abs(y[-1, 0]) < np.deg2rad(.01), 'PD should remove nominal initial error'
assert np.max(np.abs(delta)) <= p['delta_max'] + 1e-12
_, off, _ = attitude(p, 'off', linear=True)
assert abs(off[-1,0] - (p['theta0'] + p['omega0']*t[-1])) < 1e-8
flights = {mode: flight(p, mode) for mode in ['ideal','off','P','PD']}
assert max(flights['PD'][1][:,3]) > max(flights['off'][1][:,3]), 'Nominal PD should improve toy-model apogee'
p2 = dict(p, theta0=0., omega0=0.)
_, a = flight(p2, 'PD'); _, c = flight(p2, 'ideal')
assert np.max(np.abs(a-c)) < 1e-8, 'Zero disturbance must match ideal baseline'
print('PASS: stable poles, analytic response, recovery, gimbal bound, uncontrolled motion, flight benefit, zero-disturbance baseline.')
