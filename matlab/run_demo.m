function results = run_demo()
%RUN_DEMO Calculate your gains and compare off, P, PD, and ideal flight.
% Run from any folder after adding this MATLAB directory to the path.
% Copyright (c) 2026 James Bourke. MIT License.
root = fileparts(fileparts(mfilename('fullpath')));
out = fullfile(root,'results','matlab_run');
if ~exist(out,'dir'), mkdir(out); end
p = tvc_parameters();
fprintf('Generic example only: Kp = %.8f, Kd = %.8f s\n',p.Kp,p.Kd);
fprintf('b = %.8f s^-2, wn = %.8f rad/s\n',p.b,p.wn);
assignin('base','p',p); % supplies the editable p struct to the Simulink model
modes={'off','P','PD'};
figure('Name','Generic TVC attitude - simulation only');
for k=1:numel(modes)
    mode=modes{k}; a=tvc_attitude(p,mode);
    results.attitude.(mode)=a;
    subplot(2,1,1);hold on;plot(a.t,a.theta*180/pi,'LineWidth',1.6);
    subplot(2,1,2);hold on;plot(a.t,a.delta*180/pi,'LineWidth',1.6);
    writetable(table(a.t,a.theta,a.omega,a.delta,'VariableNames', ...
        {'time_s','angle_rad','rate_rad_s','gimbal_rad'}),fullfile(out,['attitude_' mode '.csv']));
    fprintf('%s: measured 2%% settling %.3f s; angle RMS %.3f deg\n',mode,a.settle,a.rms_deg);
end
subplot(2,1,1);ylabel('Angle from vertical (deg)');grid on;legend(modes);
title('Fresh generic simulation - not Project NAYA data');
subplot(2,1,2);ylabel('Gimbal command (deg)');xlabel('Time (s)');grid on;
saveas(gcf,fullfile(out,'attitude_response.png'));
figure('Name','Hypothetical planar flight');
modes={'ideal','off','P','PD'};
for k=1:numel(modes)
    mode=modes{k}; f=tvc_flight(p,mode);results.flight.(mode)=f;
    subplot(1,2,1);hold on;plot(f.t,f.y(:,4),'LineWidth',1.6);
    subplot(1,2,2);hold on;plot(f.y(:,3),f.y(:,4),'LineWidth',1.6);
    writetable(array2table([f.t f.y],'VariableNames', ...
        {'time_s','angle_rad','rate_rad_s','x_m','z_m','vx_m_s','vz_m_s'}), ...
        fullfile(out,['flight_' mode '.csv']));
    fprintf('%s: toy-model apogee %.3f m\n',mode,f.apogee);
end
subplot(1,2,1);xlabel('Time (s)');ylabel('Height (m)');grid on;legend(modes);
title('Hypothetical flight - not NAYA flight');
subplot(1,2,2);xlabel('Horizontal position (m)');ylabel('Height (m)');grid on;
saveas(gcf,fullfile(out,'flight_comparison.png'));
results.parameters=p;
save(fullfile(out,'demo_results.mat'),'results');
end
