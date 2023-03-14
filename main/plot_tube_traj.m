function plot_tube_traj(mpc,x_nom,dir,n,lb,ub)
figure; hold on
Graphics.show_convex(mpc.Xc,'Plot','yellow','FaceAlpha',0.1);
Graphics.show_convex(mpc.Xc_robust,'Plot',[0.8500 0.3250 0.0980],'FaceAlpha',0.4);
Graphics.show_convex(mpc.Xmpi_robust + mpc.sys.Z,'Plot', [0.75, 0.75, 0.75]); % rgb = [0.3, 0.3, 0.3]
Graphics.show_convex(mpc.Xmpi_robust,'Plot', [0.5, 0.5, 0.5]); % gray
Graphics.show_trajectory(x_nom(:,1:mpc.N),'b.-','LineWidth',1.5,'MarkerSize',10);
for i = 1:mpc.N
    Graphics.show_convex(x_nom(:, i)+mpc.sys.Z,'Plot', 'g', 'FaceAlpha', 0.25);
end
for k = 1:n
    load(strcat(dir,'\state_',num2str(k),'_rand_disturbance.mat'))
    plot(x(1, 1:mpc.N), x(2, 1:mpc.N),'--','LineWidth',1, 'Color',[0,0,1,0.2])
    clear('x')
end
axis([lb(1),ub(1),lb(2),ub(2)])
xlabel('$\theta$ (rad)'); ylabel('$\dot{\theta}$ (rad/s)')
set(gcf,'Units','Inches');
pos = get(gcf,'Position');
set(gcf,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
set(gcf, 'Renderer', 'Painters');

leg = legend({'$X_c$', '$X_c\ominus Z$', '$X_f \oplus Z$', '$X_f$', 'nominal traj.', 'tube'}, 'Location', 'southeast');
set(leg, 'Interpreter', 'latex');