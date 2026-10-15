# From tilt to starting gains

This is the basic idea I wanted to share from the university project: write down a simple model, choose the response you want, calculate some starting gains, and then check what the simulation actually does. You do not need NAYA's final model to learn that process.

## 1. Pick the scope and signs

We model one free-body attitude angle, theta, measured from vertical. Angular rate is omega. A positive mathematical command delta creates a positive angular acceleration. The feedback gains therefore enter with negative signs when the target angle and rate are zero.

This is a centre-of-mass free-flight rotation model. It has **no pendulum term and no pivot offset**. In a uniform gravity field, gravity acts through the centre of mass and produces no torque about it. A suspended bench model would have different equations; we do not reproduce that project-specific model here.

## 2. Turn thrust into torque

For the generic assumed moment arm ell:

$$\tau=T\ell\sin\delta,\qquad I\ddot\theta=\tau.$$

For small delta in radians, $\sin\delta\approx\delta$, giving

$$\ddot\theta=b\delta,\qquad b=T\ell/I.$$

The linear transfer function is $G(s)=b/s^2$. It describes rotational response, not altitude. The nonlinear example keeps the sine term and limits the command angle.

| Symbol | Meaning | Unit |
|---|---|---|
| theta | tilt from vertical | rad |
| omega | angular rate | rad/s |
| delta | signed abstract gimbal command | rad |
| T | thrust | N |
| ell | thrust moment arm | m |
| I | inertia about the centre of mass | kg m² |
| b | angular acceleration per command radian | s⁻² |
| Kp | angle-feedback gain | rad/rad, dimensionless |
| Kd | rate-feedback gain | s |

The moment arm and inertia are assumptions for this learning exercise. There are no CAD-derived NAYA properties in the pack.

## 3. Add proportional and derivative feedback

For zero-angle regulation:

$$\delta=-K_p\theta-K_d\dot\theta.$$

Substitute that into the linear model:

$$\ddot\theta+bK_d\dot\theta+bK_p\theta=0.$$

Compare it with the standard second-order characteristic equation:

$$s^2+2\zeta\omega_ns+\omega_n^2=0.$$

Matching coefficients gives

$$K_p=\omega_n^2/b,\qquad K_d=2\zeta\omega_n/b.$$

For positive b, Kp and Kd, the nominal linear poles are stable. P alone supplies stiffness but no damping in this undamped model, which is why its oscillation persists. Rate feedback adds damping. This is initial-condition recovery; it is not a setpoint step test. No integral gain is calculated.

## 4. Choose an approximate speed and damping

The familiar 2% settling estimate is

$$T_s\approx\frac{4}{\zeta\omega_n}.$$

Rearrange it for a starting frequency, then calculate the gains. The default exercise uses zeta = 0.8 and a settling target of 1.8 s. With the fresh example assumptions T = 12 N, ell = 0.16 m and I = 0.028 kg m²:

- b = 68.571429 s⁻²
- wn = 2.777778 rad/s
- Kp = 0.112526
- Kd = 0.064815 s

**These numbers were calculated for this pack, not copied from NAYA.**

The settling formula is approximate and intended for an underdamped linear response. Initial angular rate, saturation, nonlinear torque and the observation window all affect the measured result. The measured default response enters and stays inside ±2% of its initial angle at about 1.42 s; it need not equal the 1.8 s design estimate.

## 5. Calculate your own

```matlab
p = tvc_parameters();
p.inertia = 0.035;         % your assumed model value, kg m^2
p.zeta = 0.7;
p.settling_target = 2.2;
[p.Kp,p.Kd,p.b,p.wn] = tvc_gains(p);
r = tvc_attitude(p,'PD');
plot(r.t,r.theta*180/pi); grid on;
fprintf('Observed settling: %.3f s\n',r.settle);
```

Whenever T, ell or I changes, recalculate **b and both gains**. To use hand-picked gains, calculate b first, then overwrite p.Kp and p.Kd. `run_demo` starts again from `tvc_parameters.m`, so make persistent changes there.

## 6. Look beyond the angle plot

Plot the command too. If it sits at the angle limit, the controller is asking for more authority than the model allows. Increasing Kp further may not speed up recovery. Try a slower target and compare angular rate, peak command and settling time together.

The simple model knows nothing about sensor noise, actuator bandwidth or delays. An ideal derivative on perfect angular rate is convenient for learning; it is not a complete real-world implementation. The command is a mathematical angle, with no servo, electrical or mechanical mapping supplied.
