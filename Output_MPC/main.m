%% Clear up
clear all
close all
clc
%% Define Suspension Model
% Define LTI system and boundary
[LTI, xlb, xub, ulb, uub] = Suspension_Model();
LTI.x0 = [0;0;0;0;0;0;0;0];
LTI.d = [0;0];
LTI.Cd= [
    0, 0;
    0, 0;
    0, 5
];

LTI.Bd=[
    5, 0;
    0, 0;
    0, 0;
    0, 0;
    0, 0;
    0, 0;
    0, 0;
    0, 0
];

% Model perdictive controller parameters
dim.nx = size(LTI.A,2);     % state dimension
dim.nu = size(LTI.B,2);     % input dimension
dim.ny = size(LTI.D,1);     % output dimension
dim.nd = size(LTI.d,1);     %disturbance dimension
dim.N = 8;                  % prediction horizon

% Weight Matrix
weight.Q = diag([1e0, 1e0, 1e0, 1e0, 1e0, 1e0, 1e0,1e0]);
weight.R = diag([1e-2, 1e-2, 0, 0, 0]);
%weight.Q = 1e4*diag([1e0, 1e0, 1e0, 1e0, 1e1, 1e1, 1e0,1e0]);
%weight.R = diag([1e-4, 1e-4, 1e-4, 1e-4, 1e-4]);

% Find LQR.
[K, P] = dlqr(LTI.A, LTI.B, weight.Q, weight.R);
weight.P = P;
K = -K; % Sign convention.

%% Extended system computation

LTIe.A=[LTI.A LTI.Bd; zeros(dim.nd,dim.nx) eye(dim.nd)];
LTIe.B=[LTI.B; zeros(dim.nd,dim.nu)];
LTIe.C=[LTI.C LTI.Cd];
LTIe.D=LTI.D;
LTIe.x0=[LTI.x0; LTI.d];

%Definition of system dimension
dime.nx = dim.nx + dim.nd;     %state dimension
dime.nu=dim.nu;     %input dimension
dime.ny=dim.ny;     %output dimension
dime.N=dim.N;      %horizon


%Definition of quadratic cost function
weighte.Q=blkdiag(weight.Q,zeros(dim.nd));            %weight on output
weighte.R=weight.R;                                   %weight on input
weighte.P=blkdiag(weight.P,zeros(dim.nd));            %terminal cost
%% Compute X_f.

% Compute X_f based on exercise 4 question 2
Xn = struct();
V = struct();
Z = struct();
% Reuse code from exercise 4
[Xn.('lqr'), V.('lqr'), Z.('lqr')] = findXn(LTI.A, LTI.B, K, dim.N, xlb, xub, ulb, uub, 'lqr');
%% Preprocess of MPC

% Reuse code from exercise 3 to calculate cost
predmode=predmodgen(LTIe,dime);  
% cost = 1/2xTHx+h'x
[He,he]=costgen(predmode,weighte,dime); 

% Calculate control constraints
[A_u, b_u] = hyperrectangle(ulb, uub);
A_U = [];
b_U = [];
% A_U * U <= b_U
for i=1:dim.N
    A_U = blkdiag(A_U, A_u);
    b_U = [b_U;b_u];
end

% Calculate state constraints
% A_X * X <= b_X
[A_x, b_x] = hyperrectangle([xlb; -inf()*ones(dim.nd,1)], [xub; inf()*ones(dim.nd,1)]);
A_X = [];
b_X = [];
for i=1:dim.N+1
    A_X = blkdiag(A_X, A_x);
    b_X = [b_X;b_x];
end

% Define terminal set
terminal = Xn.lqr{1};
% x_N = T_N * x_0 + S_N * U
predmod=predmodgen(LTI,dim);  
T_N = predmod.T(end-dim.nx+1:end,:);
S_N = predmod.S(end-dim.nx+1:end,:);
%% Simulate MPC from the initial starting point.

T = 1500;    % Simulation steps
div = 15;

% Create disturbances
d1 = [zeros(1,T/div) (470/690)*0.01*0.2*ones(1,1*T/div) zeros(1,6*T/div) (470/690*0.01)*0.2*ones(1,7*T/div)];
d2 = [zeros(1, 11*T/div) (0.01)*0.2*ones(1,4*T/div)];
% Create References
yref = [zeros(3, 5*T/div), [zeros(1,10*T/div); 0.05*ones(1,10*T/div); 0.05*ones(1,10*T/div)]];

% T = 1000;    % Simulation steps
% div = 10;
% d1 = [zeros(1, 5*T/div) (470/690*0.025)*ones(1,5*T/div)];
% d2 = [zeros(1, 7*T/div) (0.01)*ones(1,3*T/div)];
% yref = [zeros(1,10*T/div); 0.05*ones(1,10*T/div); 0.05*ones(1,10*T/div)];

% Matrices to store results
xe=zeros(dime.nx,T+1);
y=zeros(dime.ny,T);
yhat=zeros(dime.ny,T);
u_rec=zeros(dime.nu,T);
xehat=zeros(dime.nx,T+1);
xr_plot = zeros(dim.nx,T);
ur_plot = zeros(dim.nu,T);

% Initial consition

xe(:,1)=LTIe.x0;
xehat(:,1)=zeros(dime.nx,1);

% Observer gain
Q_kf = 1*eye(dime.nx);
R_kf = 1*eye(dime.ny);

[~,Obs_eigvals,Obs_gain] = dare(LTIe.A',LTIe.C',Q_kf,R_kf);
Obs_gain = Obs_gain';

%% Output MPC implementation
% Receding horizon implementation
for k=1:T
    
    xe_0=xehat(:,k);  
    x_0 = [xe(1:8,k); d1(k);d2(k)];
    dhat=xehat(end-dim.nd+1:end,k);
    
    %Compute optimal ss (online, at every iteration)
    LTI.yref = yref(:,k);
    eqconstraints=eqconstraintsgen(LTI,dim,dhat);
    constraints.A = [A_x(:, 1:dim.nx), zeros(6, dim.nu);zeros(4, dim.nx), A_u];
    constraints.b = [b_x;b_u];
    [xr,ur]=optimalss(LTI,dim,weight,constraints,eqconstraints);  
    xre=[xr;dhat];
    xr_plot(:,k) = xr;
    ur_plot(:,k) = ur;
    
    % Solve the unconstrained optimization problem (with YALMIP)
    uostar = sdpvar(dime.nu*dime.N,1);          % define optimization variable
    Constraint=[uostar(3:4:end) == 0;         % Front road input
                uostar(4:4:end) == 0          % Rear road input
                uostar(5:4:end) == 0
                A_X * (predmode.T*xe_0 + predmode.S*uostar) <= b_X
                A_U * uostar <= b_U;          % Input constraints
                terminal.A * (T_N * xe_0(1:dim.nx) + S_N * uostar) <=terminal.b;        % Terminal constraints
    ];                                           % define constraints
    Objective = 0.5*uostar'*He*uostar+(he*[xe_0; xre; ur])'*uostar;    %define cost function
    optimize(Constraint,Objective);                                    %solve the problem
    uostar=value(uostar);   
    
    % Select the first input only
    u_rec(:,k)=uostar(1:dim.nu);

    % Compute the state/output evolution
    xe(:,k+1)=LTIe.A*x_0 + LTIe.B*u_rec(:,k);
    y(:,k)=LTIe.C*xe(:,k) + LTIe.D*u_rec(:,k);
    clear u_uncon
    
    % Estimated y values
    yhat(:,k) = LTIe.C*xe_0 + LTIe.D*u_rec(:,k);
        
    % Update extended-state estimation
    xehat(:,k+1)=LTIe.A*xehat(:,k)+LTIe.B*u_rec(:,k)+Obs_gain*(y(:,k)-yhat(:,k));
    
end

%% Disturbances Estimation
figure()
hold on
plot(0:T-1,d1)
plot(0:T, xehat(end-dim.nd+1,:))
%% Output States
figure()
hold on
index = 3;
plot(0:T-1, y(index,:))
plot(0:T-1, yhat(index,:))
plot(0:T-1, yref(index, :))
%% States
figure()
hold on
index = 1;
plot(0:T, xe(index,:))
plot(0:T, xehat(index,:))
plot(0:T-1, xr_plot(index,:))
%% Control inputs
figure()
hold on
index = 2;
plot(0:T-1, ur_plot(index,:))
plot(0:T-1, u_rec(index,:))