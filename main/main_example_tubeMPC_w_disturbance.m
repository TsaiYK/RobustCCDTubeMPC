clear
clc
close all

addpath('../src/')
addpath('../src/utils/')

set(0,'DefaultTextInterpreter','latex'); % change the text interpreter
set(0,'DefaultLegendInterpreter','latex'); % change the legend interpreter
set(0,'DefaultAxesTickLabelInterpreter','latex'); % change the tick interpreter
set(0, 'DefaultLineLineWidth', 2);
set(groot,'defaultAxesXGrid','on')
set(groot,'defaultAxesYGrid','on')
set(0,'defaultAxesFontSize',16)

% make your own discrete linear system with disturbance
% numerical example (2-D linear discrete-time system)
design_p_c = [1.20308043176898,-0.632149888306796,-1.22578983384540]; % Solution A
% design_p_c = [0.896239554997898,-0.844621420809160,-1.478492663481846]; % Solution B
xp = design_p_c(1);
A = [1 1; 0 1];
B = [0.5; xp]; 
Q = diag([1, 1]);
R = 0.1;
Klqr = -dlqr(A,B,Q,R); % please keep in mind that a "minus" should be included
K = design_p_c(2:3);
iniCon = [-7; -2];
N = 40; % simulation period

% bound (magnitude) of disturbances
w_mag = 0;
w_bound = [w_mag,-w_mag];
epsilon_mag = 0.1;

% construct a convex set of disturbance (2dim here)
W_vertex = []; pos = [1,1;1,2;2,2;2,1];
for ind = 1:4
    W_vertex(ind,:) = [w_bound(pos(ind,1)),w_bound(pos(ind,2))];
end

% Generating the set of B*E
BE_vertex(1,:) = B'.*[epsilon_mag, epsilon_mag];
BE_vertex(2,:) = B'.*[epsilon_mag, -epsilon_mag];
BE_vertex(3,:) = B'.*[-epsilon_mag, -epsilon_mag];
BE_vertex(4,:) = B'.*[-epsilon_mag, epsilon_mag];
BE_plus_W_vertex = W_vertex+BE_vertex;
BE_plus_W = Polyhedron(BE_plus_W_vertex);

n = 20;
for i = 1:n
    w{i} = [rand(1,N)*0.1-0.05;...
        rand(1,N)*2*epsilon_mag*xp-epsilon_mag*xp];
end

NominalLinearSystem = LinearSystem(A, B, Q, R, Klqr);
% construct disturbance Linear system
disturbance_system = DisturbanceLinearSystem(A, B, Q, R, BE_plus_W, K);
    
% compute the size of set Z
figure;
SizeZ = Graphics.show_convex(disturbance_system.Z, 'Plot', 'g', 'FaceAlpha', 0.1);
close(figure(1))

% constraints on state Xc and input Uc
lb_x = [-10,-5]; ub_x = [5,2];
Xc_vertex = [ub_x(1), ub_x(2);...
    ub_x(1), lb_x(2);...
    lb_x(1), lb_x(2);...
    lb_x(1), ub_x(2)];
Uc_vertex = [1; -1];
Xc = Polyhedron(Xc_vertex);
Uc = Polyhedron(Uc_vertex);

% create a tube_mpc simulater
% if N_horizon is too small, the path will never reach inside the robust MPI-set X_mpi_robust in time step N_horizon, then the problem becomes infeasible. 
N_horizon = 10;
mpc = TubeModelPredictiveControl(disturbance_system, Xc, Uc, N_horizon);

% The robust MPC guidances the path inside the robust MPI-set so that the path will reach the robust MPI-set in N_horizon. 
savedir_name = './results_test/';
mkdir(savedir_name);

for k = 1:n
    x(:,1) = iniCon;
    x_nom = x;
    for i = 1:N
        u_next(i) = mpc.solve(x_nom(:,i));
        u_bar(i) = mpc.solution_cache.u_nominal_seq(:,1);
        x_nom(:,i+1) = mpc.solution_cache.x_nominal_seq(:,2);
        [x(:,i+1),~] = disturbance_system.propagate(x_nom(:,i), u_next(i), w{k}(:,i)); % additive disturbance is considered inside the method 
        L(i) = x(:,i)'*Q*x(:,i)+u_next(:,i)'*R*u_next(:,i); % integrand
    end
    save(strcat(savedir_name,'state_',num2str(k),'_rand_disturbance.mat'),'x')
end
save(strcat(savedir_name,'final.mat'))

plot_tube_traj(mpc,x_nom,'results_test',n,lb_x,ub_x);
