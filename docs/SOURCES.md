# Technical sources and further reading

The equations in this pack are derived in `CONTROLS_WALKTHROUGH.md` from rotational dynamics and second-order pole matching. Code and diagrams were written afresh for this educational example. No thesis data, proprietary model or university assessment content was imported.

- NASA Glenn Research Center, **Rocket Control**. A general explanation of using thrust direction to create control torque: https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/rocket-control/
- MathWorks, **PID Controller Types for Tuning**. Background on proportional and derivative controller forms: https://www.mathworks.com/help/control/ug/pid-controller-types-for-tuning.html
- MathWorks, **Programmatic Modeling Basics**. Creating and connecting model blocks: https://www.mathworks.com/help/simulink/ug/approach-modeling-programmatically.html
- MathWorks, **add_block** and **add_line**. Native model-construction APIs used by the compatibility-focused builder: https://www.mathworks.com/help/simulink/slref/add_block.html and https://www.mathworks.com/help/simulink/slref/add_line.html
- MathWorks, **Integrator**. State integration and initial conditions: https://www.mathworks.com/help/simulink/slref/integrator.html
- MathWorks, **save_system**. Saving models as MDL or SLX: https://www.mathworks.com/help/simulink/slref/save_system.html

These references explain general concepts and APIs; they do not validate this pack's assumed parameters or trajectory. The Control System Toolbox documentation is background reading, not a dependency of the supplied MATLAB code. Some current MathWorks documentation recommends newer connection APIs; `add_line` is used here for compatibility with earlier releases.

Licence reference: Creative Commons, **Attribution 4.0 International**: https://creativecommons.org/licenses/by/4.0/

Links checked during preparation on 15 October 2026. No third-party figures or documentation text are redistributed in the pack.
