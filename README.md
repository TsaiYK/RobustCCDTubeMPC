# RobustCCD_TubeMPC
Code instruction and details of the files

# Main files (you can directly run)
1. main_p3ga & main_p3ga_satellite: run for finding non-dominated solutions. Here you have three options for design strategies: (1) control design only, (2) sequential design (plant design and control design), and (3) control co-design (CCD).
2. main_example_tubeMPC_w_disturbance & main_example_tubeMPC_w_disturbance_satellite: run for showing the actual trajectories if we add random disturbances in simulations; there are some parameters you should provide before running the codes: (1) design variables for plant and gain, (2) epsilon_mag: magnitude of control disturbance, (3) w_mag: : magnitude of external disturbance, (4) N: Time horizon, and (5) n: number of randomized disturbance sequences


# Function files (used in the main files)
1. runobjconstr_p3ga & runobjconstr_p3ga_satellite: functions for running P3GA by combining the objective functions (no additional function evaluations are needed if we have already obtained the solutions to reduce redundancy)
2. func_CCD_tube_nested & func_CCD_tube_nested_satellite: functions for evaluating the dynamics with the designed or given plant and control variables; including the processes of calculating Ak_inf, determining disturbance_system.Z, and solving the nominal MPC problem.
3. plot_tube_traj: to plot state trajectories with tubes
4. plot_ctrl_traj: to plot control trajectories
5. plot_Pareto: to plot approximate Pareto fronts for the results by P3GA and all design strategies

# Other packages
1. Solver P3GA: Galvan, E., & Malak, R. J. (2015). P3ga: An algorithm for technology characterization. Journal of Mechanical Design, 137(1), 011401.
2. MPT3: Please go to https://www.mpt3.org/Main/Installation to know how to install. After successful installation, you will see `tbxmanager' folder in your directory
3. folder `src' is based on Robust Tube-Based MPC from https://github.com/HiroIshida/robust-tube-mpc
