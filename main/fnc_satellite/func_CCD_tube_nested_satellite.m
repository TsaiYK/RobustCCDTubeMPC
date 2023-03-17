function [J,SizeZ] = func_CCD_tube_nested_satellite(xp,K)
% make your own discrete linear system with disturbance
% numerical example (2-D linear discrete-time system)
cos_alpha = xp(1); % [0,1]
d = xp(2); I = 1.744e4;
Ac = [0,1;0,0]; Bc = [0;d*cos_alpha/I];
T = 0.5; % sampling time (s)
funB = @(t) expm(Ac*t)*Bc;
A = expm(Ac*T); B = integral(funB,0,T,'ArrayValued',true);
Q = diag([1e5, 1e4]);
R = 1e-4;
iniCon = [deg2rad(25); 0];
N = 20; % simulation period
Klqr = -dlqr(A,B,Q,R); % please keep in mind that a "minus" should be included
infeasible = false;

if infeasible
    J = NaN;
    SizeZ = NaN;
else
    % bound (magnitude) of disturbances
    w_mag = 0;
    w_bound = [w_mag,-w_mag];
    epsilon_mag = 300;

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

    % construct disturbance Linear system
    disturbance_system = DisturbanceLinearSystem(A, B, Q, R, BE_plus_W, K);
    Ak_inf = disturbance_system.Ak^1e3;
    if det(Ak_inf)>1e-2
        J = NaN; SizeZ = NaN;
    else
        clear('BE_plus_W') % to clear memory because we don't use it anymore
        if isempty(disturbance_system.Z)
            J = NaN; SizeZ = NaN;
        else
            % compute the size of set Z
            SizeZ = Graphics.show_convex(disturbance_system.Z, 'DontPlot', 'g', 'FaceAlpha', 0.1);
            % close(gcf)

            %% constraints on state Xc and input Uc
            
            % for satellite
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
            clear('Xc','Uc') % to clear memory because we don't use them anymore

            x_nom(:,1) = iniCon;
            infeasible_mpc = false;
            for i = 1:N
        %         checkmemory()
                [u_next_tmp, x_nom_tmp, u_nom_tmp] = mpc.solve(x_nom(:,i));
                if isempty(u_next_tmp)
                    infeasible_mpc = true;
                    break
                else
%                     u_next(i) = u_next_tmp;
                    x_nom(:,i) = x_nom_tmp;
                    u_next(i) = min(Uc_vertex(1), max(Uc_vertex(2), u_nom_tmp(1)));
                    x_nom(:,i+1) = A*x_nom(:,i)+B*u_next(i);
                    L(i) = x_nom(:,i)'*Q*x_nom(:,i)+u_next(:,i)'*R*u_next(:,i); % integrand
                end
            end
            if infeasible_mpc
                J = NaN;
            else
                if x_nom(:,end)<=mpc.Xmpi_robust
                    J = sum(L);
                else
                    J = NaN;
                end
            end
            clear('mpc','disturbance_system')
        end
    end
    clear('mpc','disturbance_system')
end
