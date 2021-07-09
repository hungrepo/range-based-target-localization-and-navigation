%%
%% This code is used to generate simulation results for one tracker- one target in the paper entitled: 

%  Range-based navigation and target localization: observability analysis,
%  guidlines for motion planning, and filter design

%   Authors: Nguyen T. Hung, Antonio Pascoal, Institute for System and Robotic, IST, Lisbon
%   Contact: nguyen.hung@tecnico.ulisboa.pt
%   More information: https://nt-hung.github.io/research/Range-based-target-localization/

function [xhatOut,POut] = KF_Nav3D_LTV_Batista(y,xhat,P,Ts,t,u) 
% Tunning parameter of the EKF

    Q= 1e-2*diag([1, 1, 1, 0.001, 0.001, 0.01, 1, 1, 0.01]);
    R = 1;
    n=length(xhat);
    
% Discrete model
    A = [zeros(3,3) -eye(3,3)   zeros(3,1)      zeros(3,1)      zeros(3,1);
         zeros(3,3) zeros(3,3)  zeros(3,1)      zeros(3,1)      zeros(3,1);  
         u'/y       zeros(1,3)  0               -1/y            0;
         zeros(1,3) u'          0               0               -1;
         zeros(1,3) zeros(1,3)  0               0               0] ;
    B = [eye(3,3);
         zeros(3,3);
         zeros(1,3);
         zeros(1,3);
         zeros(1,3)];
    F = eye(n) + Ts*A;
    Bd = B*Ts;
    H = [zeros(1,3) zeros(1,3) 1 0 0 ] ;              
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
    xhat = F*xhat + Bd*u;
    P = F*P*F' + Q;   
% Return
    xhatOut = xhat;
    POut = P;
end