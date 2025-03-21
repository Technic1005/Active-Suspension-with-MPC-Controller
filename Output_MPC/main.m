%% Clear up
clear all
close all
clc
%% Define Suspension Model
% Define LTI system and boundary
[LTI, xlb, xub, ulb, uub] = Suspension_Model();
LTI.x0 = [0;0;0;0;0;0;0;0];
% IMU 
LTI.d=[0; 0; 0; 0];
LTI.d=[-0.1; 0.015; 0.001];
LTI.yref=[0; 0; 0; 0];
LTI.Cd= [
    1, 0, 0;
    0, 0.1, 0;
    0, 0, 1;
    0, 0, 1
];

LTI.Bd=[
    0.1, 0, 0;
    0, 0.1, 0;
    0.01, 0, 0;
    0.01, 0, 0;
    0, 0, 1;
    0, 0, 1;
    0, 0, 0;
    0, 0, 0
];

% Model perdictive controller parameters
dim.nx = size(LTI.A,2);     % state dimension
dim.nu = size(LTI.B,2);     % input dimension
dim.ny = size(LTI.D,1);     % output dimension
dim.nd = size(LTI.d,1);     %disturbance dimension
dim.N = 10;                  % prediction horizon

% Weight Matrix
weight.Q = diag([1e3, 1e3, 1e2, 1e2, 1e2, 1e2, 1e2, 1e2]);
weight.R = 1e-3*eye(4);

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
LTIe.yref=LTI.yref;

%Definition of system dimension
dime.nx = dim.nx + dim.nd;     %state dimension
dime.nu=dim.nu;     %input dimension
dime.ny=dim.ny;     %output dimension
dime.N=dim.N;      %horizon


%Definition of quadratic cost function
weighte.Q=blkdiag(weight.Q,zeros(dim.nd));            %weight on output
weighte.R=weight.R;                                   %weight on input
weighte.P=blkdiag(weight.P,zeros(dim.nd));            %terminal cost

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
[A_X, b_X] = hyperrectangle(xlb, xub);
%% Simulate MPC from the initial starting point.

T = 200;    % Simulation steps

% Matrices to store results
xe=zeros(dime.nx,T+1);
y=zeros(dime.ny,T);
u_rec=zeros(dime.nu,T);
xehat=zeros(dime.nx,T+1);

% Initial consition

xe(:,1)=LTIe.x0;
xehat(:,1)=zeros(dime.nx,1);
y(:,1)=LTIe.C*LTIe.x0;


Obs_poles = [0.7; 0.6; 0.5; 0.6; 0.8; 0.65; 0.6; 0.85; 0.65; 0.55; 0.6];
Obs_gain = place(LTIe.A', LTIe.C', Obs_poles)';

%
% Receding horizon implementation
for k=1:T
    
    xe_0=xe(:,k);  
    dhat=xehat(end-dim.nd+1:end,k);
    
    %Compute optimal ss (online, at every iteration)
    eqconstraints=eqconstraintsgen(LTI,dim,dhat);
    constraints.A = [A_X, zeros(6, dim.nu);zeros(4, dim.nx), A_u];
    constraints.b = [b_X;b_u];
    [xr,ur]=optimalss(LTI,dim,weight,constraints,eqconstraints); 
    xre=[xr;dhat];
    
    % Solve the unconstrained optimization problem (with YALMIP)
    uostar = sdpvar(dime.nu*dime.N,1);          % define optimization variable
    Constraint=[uostar(3:4:end) == 0;         % Front road input
                uostar(4:4:end) == 0          % Rear road input
                % A_X * x_0 <= b_X;            % State constraints
                A_U * uostar <= b_U;          % Input constraints
                % terminal.A * (T_N * x_0 + S_N * u_con) <=terminal.b;        % Terminal constraints
        ];                                           % define constraints
    Objective = 0.5*uostar'*He*uostar+(he*[xe_0; xre; ur])'*uostar;    %define cost function
    optimize(Constraint,Objective);                                    %solve the problem
    uostar=value(uostar);   
    
    % Select the first input only
    u_rec(:,k)=uostar(1:dim.nu);

    % Compute the state/output evolution
    xe(:,k+1)=LTIe.A*xe_0 + LTIe.B*u_rec(:,k);
    y(:,k)=LTIe.C*xe(:,k) + LTIe.D*u_rec(:,k);
    clear u_uncon
        
    % Update extended-state estimation
    xehat(:,k+1)=LTIe.A*xehat(:,k)+LTIe.B*u_rec(:,k)+Obs_gain*(y(:,k)-(LTIe.C*xehat(:,k) + LTIe.D*u_rec(:,k)));
    
end

%%
e=y-kron(ones(1,T),LTI.yref);
figure
plot(0:T-1,e),

%%
figure()
plot(0:T, xehat(end-dim.nd+2,:))
