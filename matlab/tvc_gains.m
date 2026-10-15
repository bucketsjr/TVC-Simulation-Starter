function [Kp,Kd,b,wn] = tvc_gains(p)
%TVC_GAINS Pole matching for theta_ddot = b*delta; zero-angle PD regulation.
% Gains assume angles in radians and rate in radians/second.
% Copyright (c) 2026 James Bourke. MIT License.
validateattributes(p.thrust,{'numeric'},{'scalar','real','finite','positive'});
validateattributes(p.arm,{'numeric'},{'scalar','real','finite','positive'});
validateattributes(p.inertia,{'numeric'},{'scalar','real','finite','positive'});
validateattributes(p.zeta,{'numeric'},{'scalar','real','finite','>',0,'<',1});
validateattributes(p.settling_target,{'numeric'},{'scalar','real','finite','positive'});
b = p.thrust*p.arm/p.inertia;
wn = 4/(p.zeta*p.settling_target);
Kp = wn^2/b;
Kd = 2*p.zeta*wn/b;
end
