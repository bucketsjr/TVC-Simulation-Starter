# A few things worth trying

Start with the default case and change one thing at a time. Save a copy of the parameters alongside each result so you know what produced it.

1. **Remove the damping.** Set Kd to zero and compare P with PD. Does the angle come back towards zero? Does it stay there? Explain the difference using the closed-loop equation.
2. **Slow it down.** Increase the settling target from 1.8 s to 3 s, recalculate the gains and compare peak command, observed settling and flight apogee. Is the fastest response always the one you would choose?
3. **Change the damping target.** Try zeta values of 0.5, 0.7 and 0.9 at the same settling target. Record both gains each time. Do not treat these as a physical actuator recommendation.
4. **Limit the available command.** Change delta_max to 0.5 degrees in radians. Check how long the command stays against the limit and whether the approximate pole-matching design still describes the response well.
5. **Make the model uncertain.** Design the gains for the default inertia, then simulate with inertia 25% larger while keeping the original Kp and Kd. Update b to match the actual simulated inertia without retuning the gains. Compare this with a retuned case.
6. **Remove the initial disturbance.** Set theta0 and omega0 to zero. In this idealised model, PD and the ideal vertical flight should agree. Why is that an important sanity check?

A small results table is enough:

| Run | Changed assumption | Kp | Kd | Settling | Peak command | Toy apogee | What you noticed |
|---|---|---|---|---|---|---|---|
| Default | None | | | | | | |
| Your run | | | | | | | |

If the case does not settle within the simulation window, write that rather than inventing a settling time. If a trend contradicts your first guess, check the equations and plots before changing the story. That is part of the engineering work.
