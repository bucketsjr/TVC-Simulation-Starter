function p = tvc_parameters()
%TVC_PARAMETERS Fresh educational assumptions, not Project NAYA parameters.
% Copyright (c) 2026 James Bourke. MIT License.
p.mass = 0.75;                 % kg, constant illustrative mass
p.inertia = 0.028;             % kg m^2, about the centre of mass
p.thrust = 12;                % N, constant during the example burn
p.arm = 0.16;                 % m, assumed thrust moment arm
p.zeta = 0.8;                 % desired nominal damping ratio
p.settling_target = 1.8;       % s, approximate 2 percent design target
p.delta_max = 7*pi/180;        % rad, abstract command limit
p.theta0 = 8*pi/180;           % rad, initial tilt from vertical
p.omega0 = 4*pi/180;           % rad/s, initial angular rate
p.burn = 4;                   % s, hypothetical burn duration
p.gravity = 9.81;             % m/s^2
p.density = 1.225;            % kg/m^3
p.cd_area = 0.003;            % m^2, lumped Cd times reference area
p.initial_height = 2;         % m, starts above ground without a launch rail
p.attitude_end = 6;           % s, powered attitude-only exercise
p.flight_end = 20;            % s, maximum flight integration horizon
p.max_step = 0.01;            % s, maximum ODE integration step
p.sample_dt = 0.01;           % s, saved output spacing
[p.Kp,p.Kd,p.b,p.wn] = tvc_gains(p);
end
