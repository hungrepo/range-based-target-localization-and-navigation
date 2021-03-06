%% This code is used to generate simulation results for one tracker- one target in the paper entitled: 

%  Range-based navigation and target localization: observability analysis,
%  guidlines for motion planning, and filter design

%   Authors: Nguyen T. Hung, Antonio Pascoal, Institute for System and Robotic, IST, Lisbon
%   Contact: nguyen.hung@tecnico.ulisboa.pt
%   More information: https://nt-hung.github.io/research/Range-based-target-localization/
         
%% =========================================================================================================================

function Main

% rng('default');
% ------------------------------------Initialization-------------------------------------------------------
    tf = 400;                                                             % simulation time  
    Ts = 0.1;                                                               % sampling interval  
    N = 1;                                                                  % number of tracker used
%    filter = 'KF_LTV';                                                     % EKF or EK_LTV    
% Initialize the tracker's position and orietation
    p_meas = [];
    u = [];
    u_meas = [];
    vc = [0; 0];
% Initialize the target trajectory
    Target.Depth = 0;
% The tracker' trajectory       
        rx = 50;
        ry = 50;         
        omega = 0.1;
        cx = 0.4;
        cy = 0.0;
    t = 0;
    p = [rx*sin(omega*t) + cx*t + 100; - ry*cos(omega*t) + cy];
        
% Initialize the estimate of target's trajectory       
    q_hat = [-30; -60];                                                     % Initial estimate of target position
    v_hat = [0; 0];                                                         % Initial estimate of target velocity
    
    q_hat_ekf = q_hat ;                                                     % Initial estimate of target position
    v_hat_ekf = v_hat;                                                         % Initial estimate of target velocity
    q_hat_kf = q_hat;
    v_hat_kf = v_hat;
%    r_hat = p - q_hat;
%    if (strcmp(filter,'EKF'))
        x_hat_ekf = [q_hat; v_hat];                                                 % Initial estimate of target state
        P_hat_ekf = diag([20,20, 0.1,0.1]);                                       % Initial estimate of covariance
%    elseif (strcmp(filter,'KF_LTV'))
        x_hat_kf = [norm(q_hat)^2; q_hat'*v_hat; norm(v_hat)^2;  q_hat;  v_hat];
        P_hat_kf = diag([1e5, 1e3, 1e-2, 20,20, 0.1, 0.1 ]);                                       % Initial estimate of covariance
%    end
    P_save_ekf = P_hat_ekf;
    P_save_kf = P_hat_kf;
    
% Variables to store information
    time = [];                                                              % Store time
    range = [];                                                             % Store range measurement  
    q = [];                                                                 % target position
    v = [];                                                                 % target velocity
    x_ekf = [];  x_kf = [];                                                                 % target state
    y = [];
    r = [];
    E_q_ekf = [];   E_q_kf = [];
    E_v_ekf = [];   E_v_kf = [];
%------------------------------Start simulation ---------------------------
for t = 0:Ts:tf
    time(end+1) = t;   
% Tracker trajectory and input
    p_meas(:,end+1) = p(:,end) +  0.5*randn(2,1);
%    u_meas(:,end+1) = u(:,end) + 1*randn(2,1);                           % measurement of tracker's velocity vector, assumed to be noise
% Simulate/update position and velocity of target 
   % Note: if you want to change the desired target trajectory to track, change q and v
    q(:,end+1) = [20*sin(0.01*t)+10;  0.3*t ];                             % target position
    v(:,end+1) = [0.2*cos(0.01*t); 0.3];
%     q(:,end+1) = [80*sin(0.002*t +pi);  0.0*t + 80*cos(0.002*t +pi) ];                             % target position
%     v(:,end+1) = [0.16*cos(0.002*t+pi); 0.0 - 0.16*sin(0.002*t+pi)   ];                             % target velocity
% Compute estimation error
    E_q_ekf(end+1) = norm(q(:,end)-q_hat_ekf(:,end));                             % update localization error         
    E_v_ekf(end+1) = norm(v(:,end)-v_hat_ekf(:,end));
    
    E_q_kf(end+1) = norm(q(:,end)-q_hat_kf(:,end));                             % update localization error         
    E_v_kf(end+1) = norm(v(:,end)-v_hat_kf(:,end));

% Simulate range to target     
    r(:,end+1) = p(:,end) - q(:,end); 
    range(end+1) = norm([r(:,end);Target.Depth]) + 0.5*randn;
%   x(:,end+1) = [r; v(:,end)]; 
%  y(end+1) = range(end)^2 - range(1)^2 + norm(lambda(:,end))^2;
% Estimate the target state
%    if (strcmp(filter,'EKF'))
        [x_hat_ekf(:,end+1),P_hat_ekf] = STST_EKF_PosVel_2D(range(end),p_meas(:,end),x_hat_ekf(:,end),P_hat_ekf,Ts,Target.Depth,t);
        q_hat_ekf(:,end+1) = x_hat_ekf(1:2,end); 
        v_hat_ekf(:,end+1) = x_hat_ekf(3:4,end);
        x_ekf(:,end+1) = [q(:,end);v(:,end)];
%    elseif(strcmp(filter,'KF_LTV'))
        y(end+1) = range(end)^2  - norm(p(:,end))^2; 
        [x_hat_kf(:,end+1),P_hat_kf] = STST_KF_PosVel_LTV_2D(y(end),x_hat_kf(:,end),P_hat_kf,Ts,Target.Depth,t, p(:,end));
         q_hat_kf(:,end+1) = x_hat_kf(4:5,end) ;
         v_hat_kf(:,end+1) = x_hat_kf(6:7,end);
         x_kf(:,end+1) = [norm(q(:,1))^2; q(:,1)'*v(:,end); norm(v(:,end))^2;  q(:,end);v(:,end)];                   
%    end
    P_save_ekf = [P_save_ekf P_hat_ekf];
    P_save_kf  = [P_save_kf P_hat_kf];
    
% Update tracker position
    p(:,end+1) = [rx*sin(omega*t) + cx*t + 100; - ry*cos(omega*t) + cy];

end
% -------------------------------Save Data to Workspace ------------------
    p = p';
    q = q';
    
    x_hat_ekf = x_hat_ekf';
    q_hat_ekf = q_hat_ekf';
    v_hat_ekf = v_hat_ekf';
    
    
    x_hat_kf = x_hat_kf';
    q_hat_kf = q_hat_kf';
    v_hat_kf = v_hat_kf';
    
    v = v';
    save_to_base(1);
    Plot_figure_2D;
end
%% ======================= Auxilary functions ========================================== 
