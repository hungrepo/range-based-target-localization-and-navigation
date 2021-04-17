%%
%% This code is used to generate simulation results for one tracker- one target in the paper entitled: 

%  Range-based navigation and target localization: observability analysis,
%  guidlines for motion planning, and filter design

%   Authors: Nguyen T. Hung, Antonio Pascoal, Institute for System and Robotic, IST, Lisbon
%   Contact: nguyen.hung@tecnico.ulisboa.pt
%   More information: https://nt-hung.github.io/research/Range-based-target-localization/
%%
function range_based_navigation_3D

    clear all;
    rng('default');

%% Setup for simulation

    Ts = 0.1;                             % frequency : 50Hz              as in the paper of Prof. Giovanni 
    tf = 100;                               % simulation time 
    time = [];
% beacon position
%  s = [0;0; -100];
    s = [ 2; 3; 1];
% parameter for vehicle velocity vector - this is for a new simulation
% Not for the paper of Prof. Giovani
    rx = 50; 
    ry = 50;
    rz = 50;
    omega = 0.2;
% initial vehicle position
    p0 = [2;2;0];
    p = p0;
% ocean current - assumed to be constant
%vc = [0.2;-0.3;- 0.2];    
    vc = [0.2; 0.3; -0.1];
% set up for filter
    filter = 'KF_Batista';                                 % EKF_Hung, KF_Hung, KF_Giovanni, Batista_KF 
% p_hat = [20; 10; 10];
    p_hat = [-30; 20; 30];
% vc_hat = [0;0;0];
    vc_hat = [0.1; -0.1; 0.1];
if (strcmp(filter,'EKF_Hung') )                           % for initialze EKF
    r_hat = p_hat -s;
    x_hat = [r_hat; vc_hat];
    P = 1*diag([20  20  5  1 1 1]);
elseif (strcmp(filter, 'KF_Hung'))                        % for initialize KF LTV system
    r_hat = p_hat -s;
    x_hat = [norm(r_hat)^2; r_hat'*vc_hat;norm(vc_hat)^2; r_hat; vc_hat];
    P = 1*diag([10000 1000 1 10  10  100  .5 .5 .5]);
elseif (strcmp(filter, 'KF_Giovanni'))                        % for initialize KF LTV system in Prof. Giovani paper 
    r_hat = s - p_hat;
    x_hat = [r_hat; r_hat'*vc_hat;norm(vc_hat)^2;  vc_hat];
    P = 1*diag([10  10  10 1 .1  1 1 1]);    
elseif (strcmp(filter, 'KF_Batista'))                        % for initialize KF LTV system in Prof. Batista paper 
    r_hat = s - p_hat;
    x_hat = [r_hat; vc_hat;norm(r_hat);r_hat'*vc_hat;norm(vc_hat)^2];
    P = 1*diag([20  20  5  1 1 1 100 1 .1]);        
end
% variable used to store data 
    range = [];
    u = [];             % true vehicle velocity
    u_meas = [];        % measurement of vehicle velocity 
    lambda = [0;0;0];   % integrate of the vehicle velocity 
    y_LTV = [];         % new output vector for LTV systems
    r = [];             % range 
    x = [];             % system state
%% Start simulation --------------------------------------------------------------------------
for t = 0:Ts:tf;
    time=[time t];
    range(:,end+1) = norm(p(:,end)-s) + 1*randn;                              % range measurment
%   This velocity vector for our new paper and thesis - it is more realistic than that in paper of Prof. Giovani   
    u(:,end+1) = [ rx*omega*cos(omega*t);                                   % vehicle velocity  
                  -ry*omega*sin(omega*t); 
                   2*sin(t) - 0.3];
%   This velocity vector from the paper of Prof. Giovani
%      u(:,end+1) = [ 2*cos(t);
%                    -4*sin(2*t);
%                     cos(t/2)];           
               
    u_meas(:,end+1) = u(:,end) + 1e-4*randn(3,1);                            % measurement of u with noise, parameter of noise from paper of Prof. Giovani
    if (strcmp(filter,'EKF_Hung'))
        [x_hat(:,end+1),POut] = EKF_Nav3D(range(:,end),x_hat(:,end),P,Ts,t,u_meas(:,end));  
        p_hat(:,end+1) = x_hat(1:3,end) + s;
        vc_hat(:,end+1) = x_hat(4:6,end);
        P = POut;
        
        r(:,end+1) = p(:,end) - s;            
        x(:,end+1) = [r(:,end); vc];                % store true system state
    elseif (strcmp(filter, 'KF_Hung') )
        lambda(:,end+1) = lambda(:,end) + Ts*u_meas(:,end); 
        y_LTV(end+1) = range(end)^2 + norm(lambda(:,end))^2;
        [x_hat(:,end+1),POut] = KF_Nav3D_LTV(y_LTV(:,end),x_hat(:,end),P,Ts,t,u_meas(:,end),lambda(:,end));  
        p_hat(:,end+1) = x_hat(4:6,end) + s;
        vc_hat(:,end+1) = x_hat(7:9,end);
        P = POut;
        
        r(:,end+1) = p(:,end)-s;            
        x(:,end+1) = [norm(r(:,1))^2; r(:,1)'*vc; norm(vc)^2; r(:,end); vc];  % store true system state
    elseif (strcmp(filter, 'KF_Giovanni') )
        lambda(:,end+1) = lambda(:,end) + Ts*u_meas(:,end);                         % compute Ivr in the paper of Prof. Giovanni
        y_LTV(end+1) = range(end)^2 + norm(lambda(:,end))^2 - range(1)^2 ;          % compute y_bar in the paper of Prof. Giovani
        [x_hat(:,end+1),POut] = KF_Nav3D_LTV_Giovanni(y_LTV(:,end),x_hat(:,end),P,Ts,t,u_meas(:,end),lambda(:,end));  
        p_hat(:,end+1) = s -  x_hat(1:3,end);
        vc_hat(:,end+1) = x_hat(6:8,end);
        P = POut;
        
        r(:,end+1) = s - p(:,end);            
        x(:,end+1) = [r(:,end); r(:,1)'*vc; norm(vc)^2; vc];                        % store true system state
     elseif (strcmp(filter, 'KF_Batista') )
        [x_hat(:,end+1),POut] = KF_Nav3D_LTV_Batista(range(:,end),x_hat(:,end),P,Ts,t,u_meas(:,end));  
        p_hat(:,end+1) = x_hat(1:3,end) + s;
        vc_hat(:,end+1) = x_hat(4:6,end);
        P = POut;
        
        r(:,end+1) = s - p(:,end);            
        x(:,end+1) = [r(:,end); vc; norm(r(:,end)); r(:,end)'*vc; norm(vc)^2];      % store true system state    
    end
  
    % update true vehicle trajectory by integrating with ode function 
    input = u(:,end) + vc;
    [t_ode, x_sol] = ode45(@(t,x_sol) pdot(t,x_sol,input), [0, Ts], p(:,end));
     p(:,end+1) = x_sol(end,:)';                   
end
 x = x';
 x_hat = x_hat';
 p = p';
 p_hat = p_hat';
 vc_hat = vc_hat';
 % plot vehicle trajectory 
     figure(1)
     plot3(p(:,1),p(:,2),p(:,3)); hold on;
     plot3(p_hat(:,1),p_hat(:,2),p_hat(:,3));
     plot3(s(1),s(2),s(3), '*', 'Markersize',10);                   % plot position of beacon
 % plot position estimation error
     figure(2);
     p_err = p - p_hat;
     n = length(time);
     plot(time, p_err(1:n,1), 'r', time, p_err(1:n,2), 'b', time,p_err(1:n,3), 'k');
     legend('Position error in x', 'Position error in y', 'Position error in z');
 % plot current estimation error 
     figure(3);
     vc_err = repmat(vc,1,length(vc_hat(:,1)))' - vc_hat;
     n = length(time);
     plot(time, vc_err(1:n,1), 'r', time, vc_err(1:n,2), 'b', time,vc_err(1:n,3), 'k');
     legend('Current error in x', 'Current error in y', 'Current error in z'); 
save_to_base(1);                % save to workspace
end
%% ======================= Auxilary functions ========================================== 
% vehicle kinematic
function dx = pdot(t,x,u)
    dx=u ;                      % dot_ = v + vc 
end
