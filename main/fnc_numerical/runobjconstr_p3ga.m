function [x_opt,fval,M] = runobjconstr_p3ga(xp,SaveLoc_dir,DesignStrategy,i)

xLast = []; % Last place computeall was called
myf1 = []; % Use for objective at xLast
myf2 = []; % Use for objective at xLast

%% P3GA Setting
fun1and2 = @(x) [myfun1(x) myfun2(x)];    % objective function
par = [];                          % the parameter function is in the first column of fun1and2
dom = [1,2];                          % the dominator function is in the second column of fun1and2
if strcmp(DesignStrategy,'CCD')
    K_lb = [-1.2583,-2.2467];
    K_ub = [-0.3185,-0.7033];
    lb = [0.5,K_lb];
    ub = [1.5,K_ub];
elseif strcmp(DesignStrategy,'plantdesign')
    K = [-0.6167,-1.2703];
    % For Plant Design:
    lb = 0.5;
    ub = 1.5;
elseif strcmp(DesignStrategy,'controldesign') || strcmp(DesignStrategy,'seqdesign')
    K_lb = [-1.2583,-2.2467];
    K_ub = [-0.3185,-0.7033];
    lb = K_lb;
    ub = K_ub;
else
    error('Unrecognizable design strategy!')
end

nvars = length(lb);                 % the number of design variables
Generations = 30;                  % maximum number of generations
PopulationSize = 50;                % maximum populations
nonlcon = [];
% The constraint is set empty here because we assign NaN to both objectives
% if the design is infeasible (including no infeasible solution for nominal
% MPC, no disturbance set Z exists, and AK(=A+BK) is not exponentially stable)

options = p3gaoptimset('Generations',Generations,'PopulationSize',PopulationSize,...
    'ViewProgress',true);
options.SaveLoc = SaveLoc_dir;

%% Run P3GA
[x_opt,fval,M] = p3ga(fun1and2,dom,par,nvars,[],[],[],[],lb,ub,nonlcon,options);
figure(gcf); xlabel('$J^*$','Interpreter','Latex'); ylabel('$|Z|^*$','Interpreter','Latex');

saveas(figure(gcf),strcat(options.SaveLoc,'.fig'))
saveas(figure(gcf),strcat(options.SaveLoc,'.png'))
close(gcf)

if strcmp(DesignStrategy,'seqdesign')
    save(strcat(options.SaveLoc,num2str(i),'.mat'))
else
    save(strcat(options.SaveLoc,'.mat'))
end

%% Private functions for objective
    function [f1] = myfun1(x)
        if ~isequal(x,xLast) % Check if computation is necessary
            if strcmp(DesignStrategy,'plantdesign')
                % For plant design
                xp = x;
                K = K_lqr;
            elseif strcmp(DesignStrategy,'controldesign') || strcmp(DesignStrategy,'seqdesign')
                % For Control Design
                K = x;
            else
                % For CCD
                xp = x(1);
                K = x([2,3]);
            end
            [J,SizeZ] = func_CCD_tube_nested(xp,K);
            myf1 = J;
            myf2 = SizeZ;
            xLast = x;
        end
        % Now compute objective function
        f1 = myf1;
    end


    function [f2] = myfun2(x)
        if ~isequal(x,xLast) % Check if computation is necessary
            if strcmp(DesignStrategy,'plantdesign')
                % For plant design
                xp = x;
                K = K_lqr;
            elseif strcmp(DesignStrategy,'controldesign') || strcmp(DesignStrategy,'seqdesign')
                % For Control Design
                K = x;
            else
                % For CCD
                xp = x(1);
                K = x([2,3]);
            end
            [J,SizeZ] = func_CCD_tube_nested(xp,K);
            myf1 = J;
            myf2 = SizeZ;
            xLast = x;
        end
        % Now compute objective function
        f2 = myf2;
    end
    
end