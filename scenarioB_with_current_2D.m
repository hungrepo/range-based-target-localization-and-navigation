%% This code is used to generate simulation results for one tracker- one target in the paper entitled: 

%  Range-based navigation and target localization: observability analysis,
%  guidlines for motion planning, and filter design

%   Authors: Nguyen T. Hung, Antonio Pascoal, Institute for System and Robotic, IST, Lisbon
%   Contact: nguyen.hung@tecnico.ulisboa.pt
%   More information: https://nt-hung.github.io/research/Range-based-target-localization/
         
%% =========================================================================================================================

function Main
%rng('default');
% ------------------------------------Initialization-------------------------------------------------------
    tf = 500;                                                               % simulation time  
    Ts = 0.1;                                                               % sampling interval  
    N = 1;                                                                  % number of tracker used
    filter = 1;                                                             % 1: EKF1, 2:EKF2, 3: KF for LTV, 4: KF for LTV2
    
% Initialize the tracker's position and orietation
    p = [30;-50];                                                            % tracker position  
    u = [];
    u_meas = [];
    vc = [0; 0];
% Initialize the target trajectory
    Target.Depth = 0;
% The Spatial-Temporal (ST) cure for the vehicle to track       
        rx = 50;
        ry = 50;         
        omega = 0.1;
        cx = 0.4;
        cy = 0.0;
% Initialize the estimate of target's trajectory       
    q_hat = [-100; -100];                                                     % Initial estimate of target position
    v_hat = [-0.5; 0.5];                                                    % Initial estimate of target velocity
    r_hat = p - q_hat;
    if (filter == 1)
        x_hat = [q_hat; v_hat];                                                 % Initial estimate of target state
        P_hat = diag([250,150, 0.5,0.5]);                                       % Initial estimate of covariance
    elseif (filter == 2)
        x_hat = [r_hat; v_hat];
        P_hat = diag([250,150, 0.5,0.5]);                                       % Initial estimate of covariance
    elseif (filter == 3)
        x_hat = [r_hat; r_hat'*v_hat; norm(v_hat)^2; v_hat];
        P_hat = diag([250,150, 100, .25, 0.5, 0.5]);                                       % Initial estimate of covariance
    elseif (filter == 4)
        x_hat = [r_hat; r_hat'*v_hat; norm(v_hat)^2; v_hat; norm(r_hat)^2];
        P_hat = diag([250,150, 100, .1, 0.5, 0.5, 10000]);                                       % Initial estimate of covariance
    elseif (filter == 5)
        x_hat = [q_hat; q_hat'*v_hat; norm(v_hat)^2; v_hat; norm(q_hat)^2];
        P_hat = diag([250,150, 100, 1, 0.5, 0.5, 1000]);                                       % Initial estimate of covariance
    elseif (filter == 6)
        x_hat = [r_hat; v_hat; norm(r_hat); r_hat'*v_hat; norm(v_hat)^2];
        P_hat = diag([250,150, 0.5, 0.5, 10, 10, .01]);                                       % Initial estimate of covariance
    
    end
    P_save = P_hat;
% Variables to store information
    time = [];                                                              % Store time
    E_Loc = [];                                                             % Store localization error       
    range = [];                                                             % Store range measurement  
    q = [];                                                                 % target position
    v = [];                                                                 % target velocity
    x = [];                                                                 % target state
    lambda = [];
    y = [];
    r = [];
    E_q = [];
    E_v = [];
%------------------------------Start simulation ---------------------------
for t = 0:Ts:tf
    time(end+1) = t;   
% Tracker trajectory and input
    u(:,end+1) = [rx*omega*cos(omega*t) + cx; -ry*omega*sin(omega*t) + cy]; 
    u_meas(:,end+1) = u(:,end) + 1*randn(2,1);                           % measurement of tracker's velocity vector, assumed to be noise
% Simulate/update position and velocity of target 
   % Note: if you want to change the desired target trajectory to track, change q and v
 %   q(:,end+1) = [20*sin(0.01*t +pi);  0.3*t ];                             % target position
     q(:,end+1) = [80*sin(0.002*t +pi);  0.0*t + 80*cos(0.002*t +pi) ];                             % target position
     v(:,end+1) = [0.16*cos(0.002*t+pi); 0.0 - 0.16*sin(0.002*t+pi)   ];                             % target velocity
% Compute estimation error
    E_q(end+1) = norm(q(:,end)-q_hat(:,end));                             % update localization error         
    E_v(end+1) = norm(v(:,end)-v_hat(:,end));
% Simulate range to target     
    r(:,end+1) = p(:,end) - q(:,end);
    if t==0
        range(end+1) = norm([r(:,end);Target.Depth]) + 0.5*randn;
    else
        range(end+1) = norm([r(:,end);Target.Depth]) + 0.5*randn;
    end
%   x(:,end+1) = [r; v(:,end)]; 
%  y(end+1) = range(end)^2 - range(1)^2 + norm(lambda(:,end))^2;
% Estimate the target state
    if (filter ==1)
        [x_hat(:,end+1),P_hat] = STST_EKF_PosVel(range(end),p(:,end),x_hat(:,end),P_hat,Ts,Target.Depth,t);
        q_hat(:,end+1) = x_hat(1:2,end); 
        v_hat(:,end+1) = x_hat(3:4,end);
        x(:,end+1) = [q(:,end);v(:,end)];
    elseif (filter ==2)
        [x_hat(:,end+1),P_hat] = STST_EKF2_PosVel(range(end),x_hat(:,end),P_hat,Ts,Target.Depth,t,u_meas(:,end));
         q_hat(:,end+1) =  p(:,end) - x_hat(1:2,end) ;
         v_hat(:,end+1) = x_hat(3:4,end);
         x(:,end+1) = [r(:,end);v(:,end)];
    elseif(filter == 3)
        lambda(:,end+1) = p(:,end)-p(:,1);
        y(end+1) = range(end)^2 - range(1)^2 + norm(lambda(:,end))^2; 
        [x_hat(:,end+1),P_hat] = STST_KF_PosVel_LTV_2D(y(end),x_hat(:,end),P_hat,Ts,Target.Depth,t,lambda(:,end),u_meas(:,end));
         q_hat(:,end+1) =  p(:,end) - x_hat(1:2,end) ;
         v_hat(:,end+1) = x_hat(5:6,end);
         x(:,end+1) = [r(:,end);r(:,1)'*v(:,end); norm(v(:,end))^2;v(:,end)];
    elseif(filter == 4)
        lambda(:,end+1) = p(:,end)-p(:,1);
        y(end+1) = range(end)^2 + norm(lambda(:,end))^2; 
        [x_hat(:,end+1),P_hat] = STST_KF_PosVel_LTV2_2D(y(end),x_hat(:,end),P_hat,Ts,Target.Depth,t,lambda(:,end),u_meas(:,end));
         q_hat(:,end+1) =  p(:,end) - x_hat(1:2,end) ;
         v_hat(:,end+1) = x_hat(5:6,end);
         x(:,end+1) = [r(:,end);r(:,1)'*v(:,end); norm(v(:,end))^2;v(:,end); norm(r(:,1))^2];     
    elseif(filter == 5)
     %   lambda(:,end+1) = p(:,end)-p(:,1);
        y(end+1) = range(end)^2  - norm(p(:,end))^2; 
        [x_hat(:,end+1),P_hat] = STST_KF_PosVel_LTV3_2D(y(end),x_hat(:,end),P_hat,Ts,Target.Depth,t, p(:,end));
         q_hat(:,end+1) = x_hat(1:2,end) ;
         v_hat(:,end+1) = x_hat(5:6,end);
         x(:,end+1) = [q(:,end);q(:,1)'*v(:,end); norm(v(:,end))^2;v(:,end); norm(q(:,1))^2];      
    elseif(filter == 6)
     %   lambda(:,end+1) = p(:,end)-p(:,1);
        y(end+1) = range(end); 
        [x_hat(:,end+1),P_hat] = STST_KF_PosVel_LTV4_2D(y(end),x_hat(:,end),P_hat,Ts,Target.Depth,t, u_meas(:,end));
         q_hat(:,end+1) = p(:,end) - x_hat(1:2,end) ;
         v_hat(:,end+1) = x_hat(3:4,end);
         x(:,end+1) = [r(:,end);v(:,end); norm(r(:,end)); r(:,end)'*v(:,end); norm(v(:,end))^2; ];           
    end
    P_save = [P_save P_hat];
%    E_Loc(end+1) = norm([q(:,end)-q_hat(:,end);v(:,end)-v_hat(:,end)]);                             % update localization error         
% Update tracker position
    p(:,end+1) = update_vehicle(p(:,end),u(:,end),Ts,t);  
end
% -------------------------------Save Data to Workspace ------------------
    p = p';
    q = q';
    x_hat = x_hat';
    q_hat = q_hat';
    v_hat = v_hat';
    v = v';
    save_to_base(1);
    Plot_figure_2D;
end
%% ======================= Auxilary functions ========================================== 

function x_next=update_vehicle(x_current,input,Ts,t)
    input_robot.nSteps = 4;
    input_robot.Ts=Ts;
    input_robot.u=input;
    input_robot.x = x_current;
    output_robot = RK4_integrator(@vehicle, input_robot);
    x_next = output_robot.value;
end
% vehicle dynamics
function dx = vehicle(t,x,u)
    dx=u ;
end
