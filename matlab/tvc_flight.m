function r = tvc_flight(p,mode)
%TVC_FLIGHT Hypothetical planar flight. State = [theta omega x z vx vz].
% Positive delta makes positive angular acceleration; thrust angle theta-delta.
% No NAYA flight/hardware model, no aerodynamic torque, no mass depletion.
% Copyright (c) 2026 James Bourke. MIT License.
validateattributes(p.mass,{'numeric'},{'scalar','real','finite','positive'});
validateattributes(p.burn,{'numeric'},{'scalar','real','finite','>',0,'<',p.flight_end});
y0 = [p.theta0;p.omega0;0;p.initial_height;0;0];
if strcmp(mode,'ideal'), y0(1:2)=0; end
opts = odeset('RelTol',1e-10,'AbsTol',1e-12,'MaxStep',p.max_step,'Events',@ground);
% Separate powered and coast phases at the exact burnout time.
t1 = linspace(0,p.burn,round(p.burn/p.sample_dt)+1)';
[t,y,te] = ode45(@(t,y)rhs(y,true),t1,y0,opts);
if isempty(te)
    t2 = linspace(p.burn,p.flight_end,round((p.flight_end-p.burn)/p.sample_dt)+1)';
    [tc,yc] = ode45(@(t,y)rhs(y,false),t2,y(end,:)',opts);
    t = [t;tc(2:end)]; y = [y;yc(2:end,:)];
end
r.t=t; r.y=y; r.apogee=max(y(:,4));
r.burnout_height=interp1(t,y(:,4),p.burn,'linear',NaN);
    function dy = rhs(y,powered)
        T=0; delta=0;
        if powered
            T=p.thrust; delta=tvc_command(y(1),y(2),p,mode);
        end
        drag = 0.5*p.density*p.cd_area*hypot(y(5),y(6));
        alpha = y(1)-delta;
        dy = [y(2);T*p.arm/p.inertia*sin(delta);y(5);y(6); ...
              (T*sin(alpha)-drag*y(5))/p.mass; ...
              (T*cos(alpha)-drag*y(6))/p.mass-p.gravity];
    end
    function [value,isterminal,direction] = ground(~,y)
        value=y(4); isterminal=1; direction=-1;
    end
end
