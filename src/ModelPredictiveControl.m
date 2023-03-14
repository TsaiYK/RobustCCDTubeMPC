classdef ModelPredictiveControl < handle
    
    properties (SetAccess = private)
        sys % linear sys
        optcon; % optimal contol solver object
        Xc
        Uc
        solution_cache
    end
    
    methods (Access = public)
        
        function obj = ModelPredictiveControl(sys, Xc, Uc, N)
            obj.sys = sys;
            obj.Xc = Xc;
            obj.Uc = Uc;
            obj.optcon = OptimalControler(sys, Xc, Uc, N);
            obj.solution_cache = [];
        end

        function u_next = solve(obj, x_init)
            obj.optcon.add_initial_eq_constraint(x_init);
            [x_nominal_seq, u_nominal_seq] = obj.optcon.solve();
            obj.solution_cache = struct(...
                'x_init', x_init', 'x_nominal_seq', x_nominal_seq, 'u_nominal_seq', u_nominal_seq);
            u_next = u_nominal_seq(:, 1);
        end

        function show_prediction(obj)
            assert(~isempty(obj.solution_cache), 'can be used only after solved');
            Graphics.show_convex(obj.Xc, 'Plot', 'm');
            
            Xmpi_robust = obj.sys.compute_MPIset(obj.Xc, obj.Uc);
            Graphics.show_convex(Xmpi_robust,'Plot', [0.5, 0.5, 0.5]); % gray

            x_init = obj.solution_cache.x_init;
            scatter(x_init(1), x_init(2), 50, 'bs', 'filled');

            x_nominal_seq = obj.solution_cache.x_nominal_seq;
            Graphics.show_trajectory(x_nominal_seq, 'gs-');
            
            

            leg = legend({'$X_c$', '$X_f$', 'current state', 'nominal traj.'}, 'position', [0.5 0.15 0.1 0.2]);
            set(leg, 'Interpreter', 'latex');
        end
    end
    
end
