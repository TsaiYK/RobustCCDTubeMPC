function plot_ctrl_traj(u_bar,dir,t,n)
figure; hold on
plot(t,u_bar,'b')
xlabel('t (sec)'); ylabel('u (N)')
for k = 1:n
    load(strcat(dir,'\state_',num2str(k),'_rand_disturbance.mat'))
    plot(t, u_next,'--','LineWidth',1, 'Color',[0,0,1,0.2])
    clear('u_next')
end
set(gcf,'Units','Inches');
pos = get(gcf,'Position');
set(gcf,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
set(gcf, 'Renderer', 'Painters');