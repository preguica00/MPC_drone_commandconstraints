function simulate_loop()

    close all
    clf
    hold on
    plot_prediction = plot(0,0,'or-', 'Linewidth', 1.5);
    plot_trajectory = plot(0,0,'db-','Linewidth', 1.5);
    plot_uav_body = plot(0,0,'Color',[1 0.6 0],'LineWidth',3);
    state_trajectory=[];
    control_variables=[];
    
    axis square
    xlim([0 60])
    ylim([0 60])
% % %%%%%%%%%
% xlim([0 10])
% ylim([-10 0])
%     

    [H,Ts,id_u1, id_u2,id_x,id_z,id_theta,id_dotx,id_dotz,id_dottheta] = drone_info;

    current_state = [0;0;0;0;0;0];
    current_MPC_solution = [];
    commands = [];
    
   [mass,inertia_moment,arm_moment,gravitational_acceleration_model, gravitational_acceleration_controller] = parameters;

    for k = 1:50
        
        %% Run the controller
        [command, current_MPC_solution, predicted_trajectory] = ...
            optimizetrajectory(current_state, current_MPC_solution);
        
        %% Run the simulation
        current_state = simulate_timestep(current_state, command,gravitational_acceleration_model);

        %% Visualize
        plot_prediction.XData = predicted_trajectory(:,1);
        plot_prediction.YData = predicted_trajectory(:,2);
        plot_trajectory.XData(end+1) = current_state(1);
        plot_trajectory.YData(end+1) = current_state(2);
        plot_uav_body.XData = [0 -sin(current_state(3))] + current_state(1);
        plot_uav_body.YData = [0 cos(current_state(3))]+ current_state(2);
        state_trajectory(end+1,:) = current_state;
        control_variables(end+1,:) = current_MPC_solution;
        commands(end+1,:) = command;
        
        drawnow
        pause(0.05)
        
    end
       
    figure
    plot(commands(:,1));
    hold on
    plot(commands(:,2));
    
    save('states.mat','state_trajectory')
    save('control.mat','control_variables')

end