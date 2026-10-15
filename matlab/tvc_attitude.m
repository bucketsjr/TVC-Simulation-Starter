function r = tvc_attitude(p,mode,linear)
%TVC_ATTITUDE A single free-body attitude axis; no pivot/gravity torque.
% Copyright (c) 2026 James Bourke. MIT License.
if nargin < 3, linear = false; end
validateattributes(p.delta_max,{'numeric'},{'scalar','real','finite','positive'});
t = (0:p.sample_dt:p.attitude_end)';
opts = odeset('RelTol',1e-10,'AbsTol',1e-12,'MaxStep',p.max_step);
[t,y] = ode45(@rhs,t,[p.theta0;p.omega0],opts);
r.t = t; r.theta = y(:,1); r.omega = y(:,2);
r.delta = tvc_command(r.theta,r.omega,p,mode);
% Settling is measured against 2 percent of initial angle, not zero itself.
band = 0.02*abs(p.theta0);
if band == 0, band = 1e-6; end
lastOutside = find(abs(r.theta)>band,1,'last');
if isempty(lastOutside)
    r.settle = 0;
elseif lastOutside == numel(t)
    r.settle = NaN; % never settled within the observation window
else
    r.settle = t(lastOutside+1);
end
r.rms_deg = sqrt(mean((r.theta*180/pi).^2));
    function dy = rhs(~,y)
        delta = tvc_command(y(1),y(2),p,mode);
        if linear, torqueTerm = delta; else, torqueTerm = sin(delta); end
        dy = [y(2);p.b*torqueTerm];
    end
end
