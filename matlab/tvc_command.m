function delta = tvc_command(theta,omega,p,mode)
%TVC_COMMAND Abstract PD command, with an angle limit. No hardware mapping.
% Copyright (c) 2026 James Bourke. MIT License.
switch mode
    case {'off','ideal'}
        raw = zeros(size(theta));
    case 'P'
        raw = -p.Kp*theta;
    case 'PD'
        raw = -p.Kp*theta-p.Kd*omega;
    otherwise
        error('tvc:mode','Use off, P, PD or ideal.');
end
delta = min(max(raw,-p.delta_max),p.delta_max);
end
