clear
clc
close all

%% Add paths
addpath('../src/')
addpath('../src/utils/')
addpath(genpath('C:\Users\yktsai0121\Documents\GitHub\p3ga'))
addpath(genpath('C:\Users\yktsai0121\Documents\GitHub\p3ga\dd_tools'))
addpath(genpath('C:\Users\yktsai0121\Documents\GitHub\randlc'))
addpath(genpath('C:\Users\yktsai0121\Documents\GitHub\randlc\resources\cprnd')) % for some reason this doesn't work sometimes
addpath(genpath('C:\Users\yktsai0121\Documents\GitHub\EPO'));

set(0,'DefaultTextInterpreter','latex'); % change the text interpreter
set(0,'DefaultLegendInterpreter','latex'); % change the legend interpreter
set(0,'DefaultAxesTickLabelInterpreter','latex'); % change the tick interpreter
set(0, 'DefaultLineLineWidth', 2);
set(groot,'defaultAxesXGrid','on')
set(groot,'defaultAxesYGrid','on')
set(0,'defaultAxesFontSize',14)

%% Select a Design Strategy
DesignStrategy = 'CCD';

if strcmp(DesignStrategy,'CCD')
    SaveLoc_dir = 'results\ccd_results';
    xp = [];
elseif strcmp(DesignStrategy,'plantdesign')
    SaveLoc_dir = 'results\plantdesign_results';
    xp = [];
elseif strcmp(DesignStrategy,'controldesign')
    SaveLoc_dir = 'results\controldesign_results';
    xp = 1; % if control design only, the plant design variable is fixed
elseif strcmp(DesignStrategy,'seqdesign')
    % if plant design is done and we are going to do control design based on
    % all the designed plants
    SaveLoc_dir = 'results\seqdesign_results';
    load('results\plantdesign_results\xp_opt.mat'); % from the plant design optimization results
end

if strcmp(DesignStrategy,'seqdesign')
    x_opt = []; fval = [];
    for i = 1:length(xp_opt)
        [K_opt{i},fval_iter{i},M{i}] = runobjconstr_p3ga(xp_opt(i),SaveLoc_dir,DesignStrategy,i);
        x_opt_tmp = [repmat(xp_opt(i),size(K_opt{i},1),1),K_opt{i}];
        x_opt = [x_opt;x_opt_tmp];
        fval = [fval;fval_iter{i}]; 
    end
else
    [x_opt,fval,M] = runobjconstr_p3ga(xp,SaveLoc_dir,DesignStrategy,[]);
end

%% Approximate PPD
k = 1;
nS = size(x_opt,1);
dominated_index = [];
f1 = fval(:,1);
f2 = fval(:,2);

for i = 1:nS
    for j = 1:nS
        if isempty(find(dominated_index==i))% to avoid the repeated pairs
            if ((f1(i)>f1(j) && f2(i)>f2(j)) || (f1(i)==f1(j) && f2(i)>f2(j))) % && f1(i)~=max(f1)
                % i is dominated by j
                dominated_index(k) = i;
                k = k+1;
            end
        end
    end
end

f1_PPD = f1;
f2_PPD = f2;
x_PPD = x_opt;
f1_PPD(dominated_index) = [];
f2_PPD(dominated_index) = [];
x_PPD(dominated_index,:) = [];

%% Plot PF
figure(1)
hold on
% p2 = plot(f1_PPD,f2_PPD,'ko');
xlabel('$J^*$'); ylabel('$|Z|^*$')

[f1_PPD_sort, index_sort] = sort(f1_PPD);
f2_PPD_sort = f2_PPD(index_sort);
x_PPD_sort = x_PPD(index_sort,:);

if strcmp(DesignStrategy,'CCD')
    plot(f1_PPD_sort,f2_PPD_sort,'b-*'); % CCD
    title('Design Strategy: CCD')
elseif strcmp(DesignStrategy,'plantdesign')
    plot(f1_PPD_sort,f2_PPD_sort,'-^','Color',[0.3010 0.7450 0.9330]); % only plant design
    title('Design Strategy: Plant Design')
elseif strcmp(DesignStrategy,'controldesign')
    plot(f1_PPD_sort,f2_PPD_sort,'k-o'); % control design by fixing plant
    title('Design Strategy: Control Design')
elseif strcmp(DesignStrategy,'seqdesign')
    plot(f1_PPD_sort,f2_PPD_sort,'-^','Color',[0.4660 0.6740 0.1880]); % seq design
    title('Design Strategy: Sequential Design')
end

%% Save data
save(strcat(SaveLoc_dir,'_final.mat'))
if strcmp(DesignStrategy,'PlantDesign')
    xp_opt = x_opt;
    save(strcat(SaveLoc_dir,'\xp_opt.mat'),'xp_opt')
end
