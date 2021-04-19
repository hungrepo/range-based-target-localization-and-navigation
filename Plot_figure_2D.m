n=length(time);
 
fig1 = figure(1);
set(fig1,'position',[0 0 550 400]);
%if strcmp(filter,'EKF');
h1 = plot(p(:,1),p(:,2), 'LineWidth',2,'Color','c'); hold on;
h2 = plot(p(1,1),p(1,2), '>','Markersize',10, 'LineWidth',2,'Color','c');
h3 = plot(q(:,1),q(:,2), 'LineWidth',2); 
h4 = plot(q(1,1),q(1,2), 'p', 'MarkerSize', 10,'LineWidth',2); hold on;
%end
h5 = plot(q_hat_ekf(:,1),q_hat_ekf(:,2),'-o','Color', 'r', 'MarkerSize',1);
h6 = plot(q_hat_ekf(1,1),q_hat_ekf(1,2),'o','Color', 'r', 'MarkerSize',10);

h7 = plot(q_hat_kf(:,1),q_hat_kf(:,2),'-o','Color', 'b', 'MarkerSize',1);
h8 = plot(q_hat_kf(1,1),q_hat_kf(1,2),'o','Color', 'b', 'MarkerSize',10);

xlabel('X[m]','FontSize',12,'Interpreter','latex');
ylabel('Y[m]','FontSize',12,'Interpreter','latex');
legend([h1,h2,h3,h4,h5,h7,h8],'Vehicle', 'Vehicles initial position','Target', 'Targets initial position',...,
       'EKF' , 'KF-LTV', 'Initial target position'); 
title('Trajectories of vehicle, target, and estimated targets');
% Plot position and velocity errors

fig2 = figure(2);
set(fig2,'position',[0 0 550 400]); 
subplot(2,1,1);
plot(time(1:n),E_q_ekf(1:n), 'LineWidth',1, 'Color', 'r'); hold on;
plot(time(1:n),E_q_kf(1:n), 'LineWidth',1, 'Color', 'b'); hold on;
xlabel('$t(s)$','FontSize',12,'Interpreter','latex');
ylabel('$||{\bf q} -\hat{\bf q}||$ [m]','FontSize',12,'Interpreter','latex');
legend('EKF','KF-LTV');
title('Position estimation error');
subplot(2,1,2);
plot(time(1:n),E_v_ekf(1:n), 'LineWidth',1,'Color', 'r'); hold on;
plot(time(1:n),E_v_kf(1:n), 'LineWidth',1,'Color', 'b'); hold on;
xlabel('$t(s)$','FontSize',12,'Interpreter','latex')
ylabel('$||{\bf v} -\hat{\bf v}||$ [m/s]','FontSize',12,'Interpreter','latex');
legend('EKF','KF-LTV');
title('Velocity estimation error');

