%%
function Test_HROV()
time = 200;
Ts = 0.1;
px = 10;            % initial position in X coordinate
py = 10;            % initial position in Y coordinate
psi = pi;         % vehicle heading    
k1 = 0.5;
k2 = 0.1;
k3 = 0.1;
u = [];
x = [px;py;psi];
px_d = 0;          % desired position in X
py_d = 0;          % desired position in Y    
psi_d = pi;         % desired heading 
for t=0:Ts:time

% control law
u(:,end+1) = [-k1*(px(end)-px_d)*cos(psi(end)) - k2*(py(end)-py_d)*sin(psi(end));
               k1*(px(end)-px_d)*sin(psi(end)) - k2*(py(end)-py_d)*cos(psi(end));
              -k3*(psi(end)-psi_d)];
          
% integrate vehicle kinematic               
x(:,end+1) = update_vehicle(x(:,end),u(:,end),Ts,t);  
px(end+1) = x(1,end);
py(end+1) = x(2,end);
psi(end+1) = x(3,end);
end
plot(px(1), py(1), 'o','Markersize',10, 'LineWidth',2,'Color','r'); 
hold on;
plot(px_d, py_d, '*','Markersize',10, 'LineWidth',2,'Color','b'); 
plot(px,py);
legend('initial vehicle position', 'desired position', 'travel path');

    save_to_base(1);

end
function x_next=update_vehicle(x_current,input,Ts,t)
    input_robot.nSteps = 4;
    input_robot.Ts=Ts;
    input_robot.u=input;
    input_robot.x = x_current;
    output_robot = RK4_integrator(@vehicle, input_robot);
    x_next = output_robot.value;
end
% vehicle kinematics
function dx = vehicle(t,x,u)
    ur = u(1);
    v =  u(2);
    r =  u(3);
    psi = x(3);
    dx = [ur*cos(psi) - v*sin(psi) ;
          ur*sin(psi) + v*cos(psi) ;
          r];
end