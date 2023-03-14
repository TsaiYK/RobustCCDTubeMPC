# RobustCCD_TubeMPC

Code Instruction
1.	Main files (you can directly run)

  a. main_p3ga & main_p3ga_satellite: run for finding non-dominated solutions. Here you have three options for design strategies: (1) control design only, (2) sequential design (plant design and control design), and (3) control co-design (CCD).

  b.	main_example_tubeMPC_w_disturbance & main_example_tubeMPC_w_disturbance_satellite: run for showing the actual trajectories if we add random disturbances in simulations; there are some parameters you should provide before running the codes: 
    (1) design variables for plant and gain, (2) epsilon_mag: magnitude of control disturbance, (3) w_mag: : magnitude of external disturbance, (4) N: Time horizon, and (5) n: number of randomized disturbance sequences


2.	Function files (used in the main files)

  a.	runobjconstr_p3ga & runobjconstr_p3ga_satellite: functions for running P3GA by combining the objective functions (no additional function evaluations are needed if we have already obtained the solutions to reduce redundancy)

  b.	func_CCD_tube_nested & func_CCD_tube_nested_satellite: functions for evaluating the dynamics with the designed or given plant and control variables; including the processes of calculating Ak_inf, determining disturbance_system.Z, and solving the nominal MPC problem.

  c.	plot_tube_traj: to plot state trajectories with tubes

  d.	plot_ctrl_traj: to plot control trajectories

  e.	plot_Pareto: to plot approximate Pareto fronts for the results by P3GA and all design strategies

