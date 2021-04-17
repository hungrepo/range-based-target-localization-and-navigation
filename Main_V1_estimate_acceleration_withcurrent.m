%%
% Observability for target localization
%%
function Main
clear all;
rng('default');
%% Setup for simulation
Ts=.1;
tf=100;

%% Tracker 
%  Tracker 1
a=50; r=0.5;
p10=[0;0;0];
p20=[50;50;-50];
%  Tracker 2

%% Target
%q0=[55;0;-5];          % initial position of target
q10=[-0;-20;-5];        % initial position of target 1
q20=[30;0;-10];         % initial position of target 2 
u10_T=[.5;.5;-0.2];          % initial velocity vector of target 1
u20_T=[.5;.5 ;-.5];          % initial velocity vector of target 2
a1_T=[-0.02;-0.05;-0.01];   % acceleration of target 1
a2_T=[ 0.02; 0.01;-0.01];   % acceleration of target 2


% u10_T=[.0;.0;-0.0];          % initial velocity vector of target 1
% u20_T=[.0;.0 ;-.0];          % initial velocity vector of target 2
% a1_T=[-0.00;-0.00;-0.00];   % acceleration of target 1
% a2_T=[ 0.00; 0.00;-0.00];   % acceleration of target 2
%% Ocean current
vc0=[0.2;-0.3;0.2];       % current velocity vector
%% Set up for EKF
P1_0=1*diag([10  10  10   .5 .5 .5     100,100,100,  .1, .1, .1   .01 .01, .01]);
P2_0=1*diag([10  10  10   .5 .5 .5     100,100,100,  .1, .1, .1   .01 .01, .01]);
P1=P1_0;
P2=P2_0;
range11=[];
range12=[];
range21=[];
range22=[];
%% Store data here
time=[0];
u1_T=[];            u2_T=[];
q1=[];              q2=[];
p1=[];              p2=[];
u1=[];              u2=[];
X1=[];              X2=[];
X1_hat=[];          X2_hat=[];
Target1_Est=[];     Target2_Est=[];
y1=[];              y2=[];

%% Agumented system
I3=diag([1 1 1]);
Zero3=diag([0 0 0]);
A = [Zero3     I3      Zero3   Zero3   Zero3;
     Zero3     Zero3   Zero3   Zero3   Zero3;
     Zero3     I3      Zero3   I3      Zero3;
     Zero3     Zero3   Zero3   Zero3   I3;
     Zero3     Zero3   Zero3   Zero3   Zero3]; 
B=[I3;
   zeros(12,3)]; 
C=eye(15,15);
D=zeros(15,3);
system_c=ss(A,B,C,D);
system_d=c2d(system_c,Ts);
A_d=system_d.A;
B_d=system_d.B;
X1=[p10;vc0;q10;u10_T;a1_T];
X2=[p20;vc0;q20;u20_T;a2_T];


%% initial target estimate
X1_hat=X1+1*[zeros(3,1);-vc0;[50;-50;40];-u10_T; [0.1;-0.1;0.1]]; 
X2_hat=X2+1*[zeros(3,1);-vc0;50*ones(3,1);-u20_T; 0.1*ones(3,1)]; 
Target1_Est=X1_hat(7:15,end);
Target2_Est=X2_hat(7:15,end);
u1=[0;0;0];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
%% Start simulation
for t=Ts:Ts:tf;
    
    time=[time t];
    X1(:,end+1)=A_d*X1(:,end)+B_d*u1(:,end);
    X2(:,end+1)=A_d*X2(:,end)+B_d*u1(:,end);   
    range11(:,end+1)=norm(X1(1:3,end)-X1(7:9,end))+0.1*randn;
    range12(:,end+1)=norm(p20-X1(7:9,end))+0.1*randn;
  %  y1(:,end+1)=[X1(1:3,end)+0.5*randn(3,1);range11(:,end)];
    y1(:,end+1)=[X1(1:3,end)+0.5*randn(3,1);range11(:,end);range12(:,end)];
%     y2(:,end+1)=[X2(1:3,end)+0.5*randn(3,1);range12(:,end)];
    [X1_hat(:,end+1),POut]=STST_EKF_PosVelAcc_withcurrent1(y1(:,end),X1_hat(:,end),P1,Ts,A_d,B_d,u1(:,end));  
    P1=POut;
%     [X2_hat(:,end+1),POut]=STST_EKF_PosVelAcc_withcurrent(y2(:,end),X2_hat(:,end),P2,Ts,A_d,B_d,u1(:,end));  
%     P2=POut;          
    
%    
%     Target1_Est(:,end+1)=X1_hat(7:15,end);
%     Target2_Est(:,end+1)=X2_hat(7:15,end);
    u1(:,end+1)=[a*r*cos(r*t); -a*r*sin(r*t); 0.5*sin(t)];    
%     u1(:,end+1)=[a*r*cos(r*t); -a*r*sin(r*t); .1*randn+sin(t)];    

    
%    [Target1_Est(:,end+1),POut]=MTST_EKF_PosVelAcc(range11(end),range21(end), p1(:,end),p2(:,end), Target1_Est(:,end),P1,Ts);     
%    P1=POut;
%    [Target2_Est(:,end+1),POut]=MTST_EKF_PosVelAcc(range12(end),range22(end), p1(:,end),p2(:,end), Target2_Est(:,end),P2,Ts);     
%    P2=POut;
    
end
 X1=X1';
 X2=X2';
 X1_hat=X1_hat';
 X2_hat=X2_hat';
 q1=q1';
 q2=q2';
 Target1_Est=Target1_Est';
 Target2_Est=Target2_Est';
 p1=p1';
 p2=p2';
save_to_base(1);
