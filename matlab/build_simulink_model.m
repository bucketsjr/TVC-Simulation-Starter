function mdl = build_simulink_model()
%BUILD_SIMULINK_MODEL Create the educational linear PD model as native SLX.
% Copyright (c) 2026 James Bourke. MIT License.
root=fileparts(fileparts(mfilename('fullpath')));
mdl='tvc_pd_generated';
if bdIsLoaded(mdl), error('tvc:loaded','Close tvc_pd_generated before rebuilding.'); end
new_system(mdl);
set_param(mdl,'StopTime','p.attitude_end','Solver','ode45', ...
    'RelTol','1e-8','AbsTol','1e-10','MaxStep','0.01','ReturnWorkspaceOutputs','on');
add_block('built-in/Gain',[mdl '/Angle_feedback'],'Position',[90 190 170 230],'Gain','-p.Kp');
add_block('built-in/Gain',[mdl '/Rate_feedback'],'Position',[90 85 170 125],'Gain','-p.Kd');
add_block('built-in/Sum',[mdl '/PD_sum'],'Position',[230 115 260 155],'Inputs','++');
add_block('built-in/Saturate',[mdl '/Gimbal_limit'],'Position',[310 115 380 155],'UpperLimit','p.delta_max','LowerLimit','-p.delta_max');
add_block('built-in/Gain',[mdl '/Torque_over_inertia'],'Position',[425 115 515 155],'Gain','p.b');
add_block('built-in/Integrator',[mdl '/Angular_rate'],'Position',[565 115 605 155],'InitialCondition','p.omega0');
add_block('built-in/Integrator',[mdl '/Attitude'],'Position',[660 115 700 155],'InitialCondition','p.theta0');
add_block('built-in/Mux',[mdl '/Angle_rate_command'],'Position',[780 165 785 255],'Inputs','3');
add_block('built-in/Scope',[mdl '/Response_scope'],'Position',[850 160 890 200]);
add_block('built-in/ToWorkspace',[mdl '/Simulation_log'],'Position',[835 235 945 275],'VariableName','simlog','SaveFormat','Structure With Time','MaxDataPoints','inf');
add_line(mdl,'Angle_feedback/1','PD_sum/1','autorouting','on');
add_line(mdl,'Rate_feedback/1','PD_sum/2','autorouting','on');
add_line(mdl,'PD_sum/1','Gimbal_limit/1','autorouting','on');
add_line(mdl,'Gimbal_limit/1','Torque_over_inertia/1','autorouting','on');
add_line(mdl,'Torque_over_inertia/1','Angular_rate/1','autorouting','on');
add_line(mdl,'Angular_rate/1','Attitude/1','autorouting','on');
add_line(mdl,'Angular_rate/1','Rate_feedback/1','autorouting','on');
add_line(mdl,'Attitude/1','Angle_feedback/1','autorouting','on');
add_line(mdl,'Attitude/1','Angle_rate_command/1','autorouting','on');
add_line(mdl,'Angular_rate/1','Angle_rate_command/2','autorouting','on');
add_line(mdl,'Gimbal_limit/1','Angle_rate_command/3','autorouting','on');
add_line(mdl,'Angle_rate_command/1','Response_scope/1','autorouting','on');
add_line(mdl,'Angle_rate_command/1','Simulation_log/1','autorouting','on');
save_system(mdl,fullfile(root,'simulink',[mdl '.slx']));
open_system(mdl);
fprintf('Saved %s.slx. Supply p=tvc_parameters() before simulation.\n',mdl);
end
