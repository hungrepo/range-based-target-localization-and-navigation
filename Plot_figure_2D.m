n=length(time);
if filter == 1;
    color = 'r';
elseif filter == 2
    color = 'b';
else
    color = 'm';
end
% 
fig1 = figure(1);
set(fig1,'position',[0 0 550 400]);

plot(p(:,1),p(:,2), 'LineWidth',2,'Color','c'); hold on;
plot(p(1,1),p(1,2), '>','Markersize',10, 'LineWidth',2,'Color','c');
plot(q(:,1),q(:,2), 'LineWidth',2); 
plot(q(1,1),q(1,2), 'p', 'MarkerSize', 10,'LineWidth',2); hold on;

plot(q_hat(:,1),q_hat(:,2),'-o','Color', color, 'MarkerSize',1);

plot(q_hat(1,1),q_hat(1,2),'-o','Color', color, 'MarkerSize',10);


xlabel('X[m]','FontSize',12,'Interpreter','latex')
ylabel('Y[m]','FontSize',12,'Interpreter','latex')

fig2 = figure(2);
set(fig2,'position',[0 0 550 400]); 
subplot(2,1,1);
plot(time(1:n),E_q(1:n), 'LineWidth',1, 'Color', color); hold on;
xlabel('$t(s)$','FontSize',12,'Interpreter','latex')
ylabel('$||{\bf q} -\hat{\bf q}||$ [m]','FontSize',12,'Interpreter','latex')
subplot(2,1,2);
plot(time(1:n),E_v(1:n), 'LineWidth',1,'Color', color); hold on;
xlabel('$t(s)$','FontSize',12,'Interpreter','latex')
ylabel('$||{\bf v} -\hat{\bf v}||$ [m/s]','FontSize',12,'Interpreter','latex')
