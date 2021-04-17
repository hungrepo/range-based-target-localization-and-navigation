%% This KF for the LTV system, written in the standard form used in the paper entitled

%  Range-based navigation and target localization: observability analysis,
%  guidlines for motion planning, and filter design

%   Authors: Nguyen T. Hung, Antonio Pascoal, Institute System and Robotic, IST, Lisbon
%   Contact: nguyen.hung@tecnico.ulisboa.pt
%   More information: https://nt-hung.github.io/research/Range-based-target-localization/
         
% =========================================================================================================================

function [xhatOut,POut] = STST_KF_PosVel_LTV3_2D(y,xhat,P,Ts,depth,t,p) 
% Tunning parameter of the EKF

%    Q = 1e-10*diag([.10 .1 .01 1 1 1 1]);
        Q = 1e-10*diag([10 1 .1 1 1 10 10]);
    R = 100;
    n = length(xhat);
    delta = t;
% Discrete model
    A = [zeros(1,1) zeros(1,1) zeros(1,1)  eye(1,2)   zeros(1,2) ;
         zeros(1,1) zeros(1,1) zeros(1,1)  zeros(1,2) zeros(1,2) ; 
         zeros(1,1) zeros(1,1) zeros(1,1)  zeros(1,2) zeros(1,2) ;
         zeros(2,1) zeros(2,1) zeros(2,1)  zeros(2,2) eye(2,2) ;
         zeros(2,1) zeros(2,1) zeros(2,1)  zeros(2,2) zeros(2,2) ];
     
    Ad = eye(n) + Ts*A;
%    Bd = Ts*B; 
    H = [1 2*delta delta^2 -2*p' zeros(1,2)] ;
    yhat = H*xhat; 

% Calculate the Kalman gain
    K = P*H'/(H*P*H' + R);
% Calculate the measurement residual
    resid = y - yhat;
% Update the state and covariance estimates
    Ts_meas = Ts;
    if rem(t,Ts_meas)==0
        xhat = xhat + K*resid;
        P = (eye(size(K,1))-K*H)*P;
        % only correct predicted mean and covariance when having 
        % a new range, assumed available at every Ts_meas second
    end
% Predict
   % xhat = Ad*xhat + Bd*u;
    xhat = Ad*xhat;
    P = Ad*P*Ad' + Q;
% Return    
    xhatOut = xhat;
    POut = P;