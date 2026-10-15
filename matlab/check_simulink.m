function check_simulink(mdl)
%CHECK_SIMULINK Compare Simulink output with the MATLAB linear ODE solution.
% Requires MATLAB and Simulink. Run check_demo first.
% Copyright (c) 2026 James Bourke. MIT License.
root=fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root,'simulink'));
if nargin<1, mdl='tvc_pd_educational'; end
p=tvc_parameters();
load_system(mdl);
in=Simulink.SimulationInput(mdl);
in=in.setVariable('p',p);
out=sim(in);
log=out.get('simlog');
r=tvc_attitude(p,'PD',true);
expected=interp1(r.t,[r.theta r.omega r.delta],log.time,'pchip');
actual=log.signals.values;
assert(all(size(actual)==size(expected)),'Unexpected logged signal dimensions.');
assert(max(abs(actual(:)-expected(:)))<2e-5,'Simulink/MATLAB agreement failed.');
fprintf('Simulink agrees with the linear MATLAB model within 2e-5 SI units.\n');
end
