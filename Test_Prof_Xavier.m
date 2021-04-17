%% Test the method of Prof. Xavier and our method in thesis for Scenario A (target is fixed)
%% We estimate target using batch approach (not recursive) 

clear all;
close all;
t = 0:.1:1;
n =  length(t);
q = [0;0];              % True target position to be estimated
for i = 1:n
    p(:,i) = [10*cos(2*t(i)); 10*sin(2*t(i))];  % tracker trajectory - this satisfies observability conditions
    r(i) = norm(p(:,i) - q);                    % range taken at discrete points
    y_javier(i,1) = r(i)^2 - norm(p(:,i))^2;    % build vector y in the formular of Prof. Xavier
    A_javier(i,:) = [1 -2*p(:,i)'];             % build matrix A in the formular of Prof. Xavier   
    
    lambda = p(:,i)-p(:,1);
    y_thesis(i,1) = r(i)^2 - r(1)^2 - norm(lambda)^2 ;  % build vector y in thesis eq. (4.13)
    A_thesis(i,:) = 2*lambda';                     % build matrix A in the thesis  eq. (4.14) 
end
% Now find estimated target posisition with Prof. Xavier method
    x_javier = A_javier\y_javier;                
    q_est_javier = x_javier(2:3);                             % because x := [||q||^2, q] \in R^3
    
    r_sol = A_thesis\y_thesis;                
    q_est_thesis = p(:,1)-r_sol;                             % because x := [||q||^2, q] \in R^3    

    % To make sure the method of Prof. Xavier is right we have to check two things
    % 1.   q = q_est_javier  - to make sure estimatd target is at the true position of target
    % 2.  ||q_est_javier||^2 = x_javier(1) - to make sure the constraint in x is satisfied
        
% Show on a plot

plot(p(1,:), p(2,:), 'o-', 'Linewidth', 1); hold on;
plot(q(1),q(2), 'Marker', '*', 'Markersize', 20, 'LineStyle', 'None');
plot(q_est_javier(1), q_est_javier(2), 'Marker', 'o','Markersize', 20, 'LineStyle', 'None');
plot(q_est_thesis(1), q_est_thesis(2), 'Marker', 'o','Markersize', 20, 'LineStyle', 'None');


legend('Tracker - circles are where range taken','True target','Estimated target with Javier', 'Estimated target with thesis');


%% After ploting you will see that 1. and 2. above are not satisfied. 
%  Please let me know if I make something wrong