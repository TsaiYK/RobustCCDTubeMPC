clear
% clc
close all

addpath('../src/')
addpath('../src/utils/')
    
set(0,'DefaultTextInterpreter','latex'); % change the text interpreter
set(0,'DefaultLegendInterpreter','latex'); % change the legend interpreter
set(0,'DefaultAxesTickLabelInterpreter','latex'); % change the tick interpreter
set(0, 'DefaultLineLineWidth', 2);
set(groot,'defaultAxesXGrid','on')
set(groot,'defaultAxesYGrid','on')
set(0,'defaultAxesFontSize',22)

% make your own discrete linear system with disturbance
% 1-D Satellite (2-D linear discrete-time system)
% design_p_c = [0.882062993386819,6.300786650787289,-7.178620566796799e+03,-6.082297610785315e+03]; % CCD A
design_p_c = [0.889310823260615,6.357218240319082,-7.363920086317586e+03,-7.433919302070818e+03]; % CCD B

cos_alpha = design_p_c(1); % [0,1]
d = design_p_c(2); I = 1.744e4;
Ac = [0,1;0,0]; Bc = [0;d*cos_alpha/I];
T = 0.5; % sampling time (s)
funB = @(t) expm(Ac*t)*Bc;
A = expm(Ac*T); B = integral(funB,0,T,'ArrayValued',true);
Q = diag([1e5, 1e4]);
R = 1e-4;
Klqr = -dlqr(A,B,Q,R); % please keep in mind that a "minus" should be included
K = design_p_c(3:4);
iniCon = [deg2rad(25); 0];
N = 20; % simulation period

% bound (magnitude) of disturbances
w_mag = 0;
w_bound = [w_mag,-w_mag];
epsilon_mag = 300;

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
    w{i} = [rand(1,N)*abs(BE_plus_W_vertex(1,1))*2-abs(BE_plus_W_vertex(1,1));...
        rand(1,N)*abs(BE_plus_W_vertex(1,2))*2-abs(BE_plus_W_vertex(1,2))];
end

NominalLinearSystem = LinearSystem(A, B, Q, R, Klqr);
% construct disturbance Linear system
disturbance_system = DisturbanceLinearSystem(A, B, Q, R, BE_plus_W, K);
    
% compute the size of set Z
figure;
SizeZ = Graphics.show_convex(disturbance_system.Z, 'Plot', 'g', 'FaceAlpha', 0.1);
close(figure(1))

% constraints on state Xc and input Uc
lb_x = [deg2rad(-10),-0.5]; ub_x = [deg2rad(30),1];
Xc_vertex = [ub_x(1), ub_x(2);...
    ub_x(1), lb_x(2);...
    lb_x(1), lb_x(2);...
    lb_x(1), ub_x(2)];
Uc_vertex = [3e3; -3e3];
Xc = Polyhedron(Xc_vertex);
Uc = Polyhedron(Uc_vertex);

% create a tube_mpc simulater
% if N_horizon is too small, the path will never reach inside the robust MPI-set X_mpi_robust in time step N_horizon, then the problem becomes infeasible. 
N_horizon = 10;
mpc = TubeModelPredictiveControl(disturbance_system, Xc, Uc, N_horizon);

% The robust MPC guidances the path inside the robust MPI-set so that the path will reach the robust MPI-set in N_horizon. 
savedir_name = './satellite_results_test/';
mkdir(savedir_name);

for k = 1:n
    x(:,1) = iniCon;
    x_nom = x;
    for i = 1:N
        [u_next(i), x_nom_tmp, u_nom_tmp] = mpc.solve(x(:,i));
        x_nom(:,i+1) = mpc.solution_cache.x_nominal_seq(:,2);
        u_bar(i) = mpc.solution_cache.u_nominal_seq(:,1);
        [x(:,i+1),~] = disturbance_system.propagate(x(:,i), u_next(i), w{k}(:,i)); % additive disturbance is considered inside the method 
        L(i) = x(:,i)'*Q*x(:,i)+u_next(:,i)'*R*u_next(:,i); % integrand
    end
    save(strcat(savedir_name,'state_',num2str(k),'_rand_disturbance.mat'),'x','u_next')
end
save(strcat(savedir_name,'final.mat'))

plot_tube_traj(mpc,x_nom,'satellite_results_test',n,lb_x,ub_x);

plot_ctrl_traj(u_bar,'satellite_results_test',(0:(N-1))*T,n)
