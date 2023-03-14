classdef TubeModelPredictiveControl < handle
    
    properties (SetAccess = public)
        sys % linear system with disturbance
        optcon; % optimal contol solver object
        Xc; % state constraint
        Uc; % input constraint
        Xc_robust; % Xc-Z (Pontryagin diff.)
        Xmpi_robust; % MPI set. Note that this is made using Xc-Z and Uc-KZ (instead of Xc and Uc)
        N; % horizon
        solution_cache
    end
    
    methods (Access = public)
        function obj = TubeModelPredictiveControl(sys, Xc, Uc, N)
            %----------approximation of d-inv set--------%
            %create robust X and U constraints, and construct solver using X and U
            Xc_robust = Xc - sys.Z;
            Uc_robust = Uc - sys.K*sys.Z;
            %optcon.reconstruct_ineq_constraint(Xc_robust, Uc_robust)
            optcon = OptimalControler(sys, Xc_robust, Uc_robust, N);

            %robustize Xmpi set and set it as a terminal constraint
            Xmpi_robust = sys.compute_MPIset(Xc_robust, Uc_robust);
            optcon.add_terminal_constraint(Xmpi_robust);

            %fill properteis
            obj.sys = sys;
            obj.optcon = optcon;
            obj.Xc = Xc;
            obj.Uc = Uc;
            obj.Xc_robust = Xc_robust;
            obj.Xmpi_robust = Xmpi_robust;
            obj.N = N;
            obj.solution_cache = [];
        end

        function [u_next, x_nom, u_nominal] = solve(obj, x_init)
            % Note that solution of optimal controler is cached in obj.solution_cache
            Xinit = x_init + obj.sys.Z;
            obj.optcon.add_initial_constraint(Xinit);
            [x_nominal_seq, u_nominal_seq, infeasible] = obj.optcon.solve();

            if infeasible
                u_next = []; x_nom = []; u_nominal = [];
            else
                obj.solution_cache = struct(...
                    'x_init', x_init', 'x_nominal_seq', x_nominal_seq, 'u_nominal_seq', u_nominal_seq);

                u_nominal = u_nominal_seq(:, 1);
                u_feedback = obj.sys.K * (x_init - x_nominal_seq(:, 1));
                x_nom = x_nominal_seq(:, 1);
                u_next = u_nominal + u_feedback;
            end
        end

        function [] = show_prediction(obj)
            assert(~isempty(obj.solution_cache), 'can be used only after solved');
            Graphics.show_convex(obj.Xc,'Plot','yellow','FaceAlpha',0.1);
            Graphics.show_convex(obj.Xc_robust,'Plot',[0.8500 0.3250 0.0980],'FaceAlpha',0.4);
            Graphics.show_convex(obj.Xmpi_robust + obj.sys.Z,'Plot', [0.75, 0.75, 0.75]); % rgb = [0.3, 0.3, 0.3]
            Graphics.show_convex(obj.Xmpi_robust,'Plot', [0.5, 0.5, 0.5]); % gray
            x_init = obj.solution_cache.x_init;
            scatter(x_init(1), x_init(2), 50, 'ks', 'filled');
            x_nominal_seq = obj.solution_cache.x_nominal_seq;
            Graphics.show_trajectory(x_nominal_seq,'b.-','LineWidth',1.5,'MarkerSize',10);
            for j=1:obj.N+1
                Graphics.show_convex(x_nominal_seq(:, j)+obj.sys.Z,'Plot', 'g', 'FaceAlpha', 0.25);
            end

%             leg = legend({'$X_c$', '$X_c\ominus Z$', '$X_f \oplus Z$', '$X_f (X_{mpi})$', 'current state', 'nominal traj.', 'tube'}, 'position', [0.5 0.15 0.1 0.2]);
            leg = legend({'$X_c$', '$X_c\ominus Z$', '$X_f \oplus Z$', '$X_f (X_{mpi})$', 'initial state', 'nominal traj.', 'tube'}, 'Location', 'southwest');
            set(leg, 'Interpreter', 'latex');
        end
    end
end
