function cost = costfunction(y, H)

    [H, Ts, id_u1, id_u2,id_x,id_z,id_theta,id_dotx,id_dotz,id_dottheta] = drone_info;
   [mass,inertia_moment,arm_moment,gravitational_acceleration_model, gravitational_acceleration_controller] = parameters;
%     costt=[];
    % set points 
    x_final=60;
    z_final=60;
%     common_final = mass*gravitational_acceleration_controller;
    common_final=0;
    % Unpacking 
    mode_diff = y(id_u1);
    mode_common = y(id_u2);
    x = y(id_x);
    z = y(id_z);
    theta = y(id_theta);
    x_velocity = y(id_dotx);
    z_velocity = y(id_dotz);
    angular_velocity = y(id_dottheta);

    cost =  sum(2*(x(:)-x_final).^2+(z(:)-z_final).^2 +80*(theta(:)).^2 + (x_velocity(:)).^2+ (z_velocity(:)).^2+2*(angular_velocity(:)).^2 +80*(mode_diff(:)).^2+(mode_common(:)-common_final).^2);

%     costt(end+1,:) = cost;
%     save('cost.mat','costt')

end