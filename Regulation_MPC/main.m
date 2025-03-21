%% Clear up
clear all
close all
clc
%% Define Suspension Model
% Define LTI system and boundary
[LTI, xlb, xub, ulb, uub] = Suspension_Model();

% Model perdictive controller parameters
dim.nx = size(LTI.A,2);     % state dimension
dim.nu = size(LTI.B,2);     % input dimension
dim.ny = size(LTI.D,1);     % output dimension
dim.N = 5;                  % prediction horizon

% Weight Matrix
weight.Q = diag([1e4, 1e4, 1e3, 1e3, 1e6, 1e6,1e3,1e3]);
weight.R = diag([1e-4, 1e-4, 1e-3, 1e-3, 1e-3]);

% Find LQR.
[K, P] = dlqr(LTI.A, LTI.B, weight.Q, weight.R);
weight.P = P;
K = -K; % Sign convention.

%% Compute X_f.

% Compute X_f based on exercise 4 question 2
Xn = struct();
V = struct();
Z = struct();
% Reuse code from exercise 4
[Xn.('lqr'), V.('lqr'), Z.('lqr')] = findXn(LTI.A, LTI.B, K, dim.N, xlb, xub, ulb, uub, 'lqr');

%% Preprocess of MPC

% Reuse code from exercise 3 to calculate cost
predmod=predmodgen(LTI,dim);     
% cost = 1/2xTHx+h'x
[H,h]=costgen(predmod,weight,dim);

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

% Define terminal set
terminal = Xn.lqr{1};
% x_N = T_N * x_0 + S_N * U
T_N = predmod.T(end-dim.nx+1:end,:);
S_N = predmod.S(end-dim.nx+1:end,:);
%% Simulate MPC from the initial starting point.

T = 200;    % Simulation steps

% Matrices to store results
x=zeros(dim.nx,T+1);
u_rec=zeros(dim.nu,T);

% Initial consition
LTI.x0 = [-0.05;0.3;2;-2;0.01;-0.012;0.002;-0.002];
x(:,1)=LTI.x0;

%% Without MPC Controller
x_original = LTI.x0;
u_original = [0;0;0;0;2300];
for k=1:T
    x_new = LTI.A * x_original(:,k) + LTI.B * u_original;
    x_original = [x_original, x_new];
end
%%
% Receding horizon implementation
for k=1:T
    
    x_0=x(:,k);
    
    % Solve the unconstrained optimization problem (with YALMIP)
    u_con = sdpvar(dim.nu*dim.N,1);          % define optimization variable
    Constraint=[u_con(3:5:end) == 0;         % Front road input
                u_con(4:5:end) == 0          % Rear road input
                u_con(5:5:end) == 2300          % Pitch Moment
                A_X * x_0 <= b_X;            % State constraints
                A_U * u_con <= b_U;          % Input constraints
                terminal.A * (T_N * x_0 + S_N * u_con) <=terminal.b;        % Terminal constraints
        ];                                           % define constraints
    Objective = 0.5*u_con'*H*u_con+(h*x_0)'*u_con;     % define cost function
    optimize(Constraint,Objective);                          % solve the problem
    u_con=value(u_con);                                  % assign the solution
    
    % Select the first input only
    u_rec(:,k)=u_con(1:5);

    % Compute the state/output evolution
    x(:,k+1)=LTI.A*x_0 + LTI.B*u_rec(:,k);
    clear u_con
    
end

%%
step = 0:1:T;
index = 6;
figure()
hold on
stairs(step, x(index,:)) % 用阶梯图画出状态保持器的效果
stairs(step, x_original(index,:))
xlabel('Step')
ylabel('State x1')
%%
figure()
stairs(step(1:end-1), u_rec(1,:)) % 用阶梯图画出状态保持器的效果
xlabel('Step')
ylabel('Input u1')