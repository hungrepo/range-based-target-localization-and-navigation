fig1=figure(1);
% set(fig1,'position',[0 0 500 350]);
plot3(X1(:,1),X1(:,2),X1(:,3), 'b','linewidth',1);
hold on;
plot3(X1(:,7),X1(:,8),X1(:,9), 'r', 'linewidth',1);
plot3(X1_hat(:,7),X1_hat(:,8),X1_hat(:,9), 'r-.', 'linewidth',1);


plot3(X1_hat(1,7),X1_hat(1,8),X1_hat(1,9), 'bo', 'linewidth',1);

e1=X1-X1_hat;
% plot current error
fig2=figure(2);
set(fig2,'position',[0 0 300 250]);
plot(time,e1(:,4),'linewidth',1);
hold on
plot(time,e1(:,5),'linewidth',1);
plot(time,e1(:,6),'linewidth',1);


% plot position error
fig3=figure(3);
set(fig3,'position',[0 0 300 250]);
plot(time,e1(:,7),'linewidth',1);
hold on
plot(time,e1(:,8),'linewidth',1);
plot(time,e1(:,9),'linewidth',1);

% plot velocity error
fig4=figure(4);
set(fig4,'position',[0 0 300 250]);
plot(time,e1(:,10),'linewidth',1);
hold on
plot(time,e1(:,11),'linewidth',1);
plot(time,e1(:,12),'linewidth',1);

% plot acceleration error
fig5=figure(5);
set(fig5,'position',[0 0 300 250]);
plot(time,e1(:,13),'linewidth',1);
hold on
plot(time,e1(:,14),'linewidth',1);
plot(time,e1(:,15),'linewidth',1);


