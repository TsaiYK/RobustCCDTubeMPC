clear
clc
close all

% % Numerical Example
% CD = load('..\final_data\numerical\controldesign_p3ga_result.mat');
% PD = load('..\final_data\numerical\plantdesign_p3ga_result.mat');
% SeqD = load('..\final_data\numerical\seq_design_results_w_pd_result.mat');
% CCD = load('..\final_data\numerical\CCD_p3ga_result.mat');
% selected_index = [17,7];
% selected_SeqD_design = SeqD.x_PPD(selected_index(1),:);
% selected_CCD_design = CCD.x_PPD(selected_index(2),:);

% % Satellite
CD = load('results_satellite2\controldesign_results_final.mat');
PD = load('results_satellite2\plantdesign_results_final.mat');
SeqD = load('results_satellite2\seqdesign_results_final.mat');
CCD = load('results_satellite2\ccd_results_final.mat');

% CD = load('..\final_data\satellite\controldesign_p3ga_result.mat');
% PD = load('..\final_data\satellite\plantdesign_p3ga_result.mat');
% SeqD = load('..\final_data\satellite\seq_design_results_w_pd_result.mat');
% CCD = load('..\final_data\satellite\ccd_p3ga_result.mat');

figure
hold on
p1 = plot(CD.f1_PPD_sort,CD.f2_PPD_sort,'k-o'); % control design by fixing plant
p2 = plot(PD.f1_PPD_sort,PD.f2_PPD_sort,'-^','Color',[0.3010 0.7450 0.9330]); % only plant design
p3 = plot(CCD.f1_PPD_sort,CCD.f2_PPD_sort,'b-*'); % CCDxlabel('$J^*$'); ylabel('$|Z|$')
legend('Control Design','Plant Design','CCD')
xlabel('$V^*_{20}$','Interpreter','Latex'); ylabel('$\bf{|Z|}^*$','Interpreter','Latex');
figure
hold on
p1 = plot(CD.f1_PPD_sort,CD.f2_PPD_sort,'k-o'); % control design by fixing plant
p2 = plot(SeqD.f1_PPD_sort,SeqD.f2_PPD_sort,'-^','Color',[0.4660 0.6740 0.1880]); % seq design
p3 = plot(CCD.f1_PPD_sort,CCD.f2_PPD_sort,'b-*'); % CCDxlabel('$J^*$'); ylabel('$|Z|$')
legend('Control Design','Seq. Design','CCD')
xlabel('$V^*_{20}$','Interpreter','Latex'); ylabel('$\bf{|Z|}^*$','Interpreter','Latex');
