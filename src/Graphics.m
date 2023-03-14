classdef Graphics<handle
    % This is a function collection rather than a class
    % So, All methods are static.
    % plot trajectory and show convex set which are projected from n-dim
    
    methods (Static)
        function SizeZ = show_convex(P, PlotOrNot, varargin)
            P_reduced = projectPolytope2Plane(P);
            if strcmp(PlotOrNot,'Plot')
                fill(P_reduced.V(:, 1), P_reduced.V(:, 2), varargin{:});
%                 cm = colororder;
%                 hatchfill2(hp,'single','HatchAngle',0,'hatchcolor',cm(1,:));
                hold on;
            end
            SizeZ = polyarea(P_reduced.V(:,1),P_reduced.V(:,2));
            
        end

        function show_trajectory(x_seq, varargin)
            plot(x_seq(1, :), x_seq(2, :), varargin{:})
            hold on;
        end

    end
end

function P_projected = projectPolytope2Plane(P)
    vert = P.V;
    x_vert = round(vert(:, 1), 5);
    y_vert = round(vert(:, 2), 5);
    idx = convhull(x_vert, y_vert);
    P_projected = Polyhedron([x_vert(idx), y_vert(idx)]);
end
