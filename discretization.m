function [c, ceq] = discretization(y,x_init,z_init,theta_init,xvelocity_init,zvelocity_init,angvelocity_init)

[H,Ts,id_u1, id_u2,id_x,id_z,id_theta,id_dotx,id_dotz,id_dottheta] = drone_info;
[mass,inertia_moment,arm_moment,gravitational_acceleration_model, gravitational_acceleration_controller] = parameters;

% Unpacking
x = y(id_x);
z = y(id_z);
theta = y(id_theta);
x_velocity = y(id_dotx);
z_velocity = y(id_dotz);
angular_velocity = y(id_dottheta);
mode_diff = y(id_u1);
mode_common = y(id_u2);


current_state=[x_init; z_init;theta_init;xvelocity_init; zvelocity_init;angvelocity_init];
state =[current_state, [x,z,theta,x_velocity,z_velocity,angular_velocity]'];
control=[mode_diff,mode_common];


% Run discrete prediction
ceq = [];
for i = 1:H
    ceq = [ceq; (state(:,i+1) - timestep_euler(Ts,state(:,i), control(i,:),gravitational_acceleration_controller))];

end

c=[];
% c=vecnorm(abs(theta)-pi/9);

end