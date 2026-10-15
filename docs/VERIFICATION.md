# What has been checked

## Executed numerical reference

The bundled figures and reference CSVs were computed using Python, NumPy, SciPy and Matplotlib. Both powered attitude and flight use the documented nonlinear sine torque. The Simulink exercise intentionally uses the linear small-command approximation.

From the repository root:

```bash
python -m pip install -r verification/requirements.txt
python verification/check_reference.py
python verification/reference_simulation.py
python verification/release_check.py
```

The small check script verifies stable nominal PD poles, linear ODE agreement with a matrix-exponential solution to within 1e-7, nominal attitude recovery, the command-angle bound, the analytical uncontrolled response, the selected flight's PD/off ordering, and zero-disturbance agreement with the ideal vertical reference. These checks establish mathematical consistency for this selected model, not experimental validation.

The numerical solver is `solve_ivp` with relative tolerance 1e-10, absolute tolerance 1e-12 and maximum step 0.01 s. Output is saved every 0.01 s. Flight is integrated in separate powered and coast phases, so the exact four-second thrust discontinuity is not crossed by one integration step. Ground contact terminates the trajectory. The resulting apogee metric is the maximum sampled height.

An additional release check halves maximum integration step and output spacing and compares the default PD/off apogees and PD settling time. Its results are in `results/verification_report.txt`. This checks time-resolution sensitivity for the sample cases; it is not a general model validation.

## MATLAB checks supplied

```matlab
addpath('matlab');
check_demo;
```

This checks the MATLAB implementation against its analytical linear response and the same basic physical sanity checks. It needs base MATLAB only. It has not been executed in the preparation environment because MATLAB was unavailable.

## Simulink checks supplied

```matlab
p = tvc_parameters();
addpath('simulink');
check_simulink;
```

The model contains negative angle and rate feedback, a sum, command saturation, torque/inertia gain, two integrators, a scope and a three-signal workspace log. It reads the `p` structure; it contains no embedded hardware interface or project-specific parameter set.

`check_simulink` supplies fresh default parameters with `Simulink.SimulationInput`, then compares the logged angle, angular rate and command with the linear MATLAB ODE using a 2e-5 absolute tolerance after interpolation. That comparison is linear-to-linear, not linear-to-nonlinear.

The included `.mdl` file is hand-authored legacy-format Simulink source. It has **not** been loaded or compiled in Simulink here. The alternative `build_simulink_model` script creates the same loop using native blocks and saves `simulink/tvc_pd_generated.slx`. That builder has also not been executed here. If your release has trouble loading the legacy source, use the builder and run `check_simulink('tvc_pd_generated')`.

A static check inspected block connections, input ports, source/destination names and balanced model braces. That is useful for catching transcription errors but is not proof that Simulink accepts the file. Keep this distinction when presenting the pack.

## Interpretation limits

The nominal linear analytic solution is a verification reference. No experiment validates the generic plant or the flight extension. Do not describe this pack as a validated NAYA digital twin, a recovered thesis model, a flight-tested controller or a reproduced powered test.
