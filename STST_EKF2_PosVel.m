%% This KF for the LTV system, written in the standard form used in the paper entitled

%  Range-based navigation and target localization: observability analysis,
%  guidlines for motion planning, and filter design

%   Authors: Nguyen T. Hung, Antonio Pascoal, Institute System and Robotic, IST, Lisbon
%   Contact: nguyen.hung@tecnico.ulisboa.pt
%   More information: https://nt-hung.github.io/research/Range-based-target-localization/
         
% =========================================================================================================================

function [xhatOut,POut] = STST_EKF2_PosVel(y,xhat,P,Ts,depth,t, u) 
% Tunning parameter of the EKF
    Q = 1e-6*diag([10 10 1 1]);
    R = 10;
    n = length(xhat);
% Discrete model
    A = [zeros(2,2)   -eye(2,2);
         zeros(2,2)   zeros(2,2)];
    B = [eye(2,2);
         zeros(2,2)];
     
    Ad = eye(n) + Ts*A;
    Bd = Ts*B; 
    rhat = xhat(1:2);
    H = [rhat'/norm(rhat)  zeros(1,2)] ;
    yhat = norm(rhat); 

% Calculate the Kalman gain
    K = P*H'/(H*P*H' + R);
% Calculate the measurement residual
    resid = y - yhat;
% Update the state and covariance estimates
    Ts_meas = 1;
    if rem(t,Ts_meas)==0
        xhat = xhat + K*resid;
        P = (eye(size(K,1))-K*H)*P;
        % only correct predicted mean and covariance when having 
        % a new range, assumed available at every Ts_meas second
    end
% Return
% Predict
    xhat = Ad*xhat + Bd*u;
    P = Ad*P*Ad' + Q;
    xhatOut = xhat;
    POut = P;