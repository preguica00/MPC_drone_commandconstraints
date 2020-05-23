# MPC_drone_commandconstraints


This example is a quadcopter model controlled by an MPC controller.
To run the program, use `simulate_loop.m`.

----------------------------------------------------------------------------------
The MPC controller corresponds to the `optimize_trajectory.m`.
The controller is implemented with fmincon, which has a cost function and a corresponding nonlinear 
constraints function called `discretization.m` This last function uses `timestep_euler` to implement the dynamics constraint.

-----------------------------------------------------------------------
The `quadcopter_ode.m` corresponds to the model equations. The simulation is made with `simulate_timestep.m` 
that uses `quadcopter_ode.m`

------------------------------------------------------------
To plot the states and the control use `simulate_variables.m`, that loads `states.mat` and `control.mat`.
