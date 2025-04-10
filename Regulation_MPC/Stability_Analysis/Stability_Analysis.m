%% Clear up
clear all
clc
%% Define Suspension Model
% Define LTI system and boundary
[LTI, xlb, xub, ulb, uub] = Suspension_Model();

% Model perdictive controller parameters
dim.nx = size(LTI.A,2);     % state dimension
dim.nu = size(LTI.B,2);     % input dimension
dim.ny = size(LTI.D,1);     % output dimension
dim.N = 8;                  % prediction horizon

% Weight Matrix
weight.Q = diag([1e4, 1e4, 1e3, 1e3, 1e6, 1e6,1e3,1e3]);
%weight.Q = diag([1e5, 1e5, 1e5, 1e5, 1e5, 1e5,1e5,1e5]);
weight.R = diag([1e-4, 1e-4, 1e-3, 1e-3, 1e-3]);
%weight.R = diag([1e-3, 1e-3, 1e-3, 1e-3, 1e-3]);
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
[A_x, b_x] = hyperrectangle(xlb, xub);
A_X = [];
b_X = [];
for i=1:dim.N+1
    A_X = blkdiag(A_X, A_x);
    b_X = [b_X;b_x];
end

% Define terminal set
terminal = Xn.lqr{1};
% x_N = T_N * x_0 + S_N * U
T_N = predmod.T(end-dim.nx+1:end,:);
S_N = predmod.S(end-dim.nx+1:end,:);
%% Simulate MPC from the initial starting point.

lb = -0.5;
ub = 0.5;
res = 0.1;

dotx_b = lb:res:ub;
dottheta = lb:res:ub;
mat = zeros(length(dotx_b),length(dottheta));
for i = 1:length(dotx_b)
    for j = 1:length(dottheta)
        T = 2;    % Simulation steps
        % Matrices to store results
        x=zeros(dim.nx,T+1);
        u_rec=zeros(dim.nu,T);

        % Initial consition
        LTI.x0 = [dotx_b(i);dottheta(j);0;0;0;0;0;0];
        x(:,1)=LTI.x0;

        % Receding horizon implementation
        for k=1:T
            
            x_0=x(:,k);
            
            % Solve the unconstrained optimization problem (with YALMIP)
            options = sdpsettings('solver', 'quadprog', 'verbose', 1);
            u_con = sdpvar(dim.nu*dim.N,1);          % define optimization variable
            Constraint=[u_con(3:5:end) == 0;         % Front road input
                        u_con(4:5:end) == 0          % Rear road input
                        u_con(5:5:end) == 0          % Pitch Moment
                        A_X * (predmod.T*x_0 + predmod.S*u_con) <= b_X;             % State constraints
                        A_U * u_con <= b_U;          % Input constraints
                        terminal.A * (T_N * x_0 + S_N * u_con) <=terminal.b;        % Terminal constraints
                ];                                           % define constraints
            Objective = 0.5*u_con'*H*u_con+(h*x_0)'*u_con;     % define cost function
            optimize(Constraint,Objective,options);                          % solve the problem
            u_con=value(u_con);                                  % assign the solution
            
            % Select the first input only
            u_rec(:,k)=u_con(1:5);
        
            % Compute the state/output evolution
            x(:,k+1)=LTI.A*x_0 + LTI.B*u_rec(:,k);
            %clear u_con
            
        end
        inSet = all(terminal.A * x(:,end) <= terminal.b);
        mat(i, j) = inSet;
    end
    disp(i)
end

%% Plot
N2 = load("Neq2.mat");
N8 = load("Neq8.mat");

[row, col] = size(N2.mat);
[X, Y] = meshgrid(N2.dottheta, N2.dotx_b); % Grid
resX = 0.1;
resY = 0.1;
figure(1);
subplot(1,2,1)
pbaspect([1 1 1]);
hold on;

for i = 1:row
    for j = 1:col
        theta = X(i, j);
        xb = Y(i, j);
        color = [1 0 0];
        if N2.mat(i, j) == 1
            color = [0 1 0];
        end
        rectangle('Position', [theta - resX/2, xb - resY/2, 0.085, resY], ...
                  'FaceColor', color, 'EdgeColor', 'None');
    end
end
xlim([min(N2.dottheta)-resX/2, max(N2.dottheta)+resX/2]);
ylim([min(N2.dotx_b)-resY/2, max(N2.dotx_b)+resY/2]);
xlabel('$\dot{\theta}$[rad/s]', 'Interpreter', 'latex');
ylabel('$\dot{x_b}$[m/s]', 'Interpreter', 'latex');
title('Estimation of $\mathcal{X}_N$ (N=2)', 'Interpreter', 'latex');

subplot(1,2,2)
pbaspect([1 1 1]);
[row, col] = size(N8.mat);
[X, Y] = meshgrid(N8.dottheta, N8.dotx_b);
resX = 0.1;
resY = 0.1;
for i = 1:row
    for j = 1:col
        theta = X(i, j);
        xb = Y(i, j);
        color = [1 0 0];
        if N8.mat(i, j) == 1
            color = [0 1 0];
        end
        rectangle('Position', [theta - resX/2, xb - resY/2, 0.085, resY], ...
                  'FaceColor', color, 'EdgeColor', 'None');
    end
end
xlim([min(N8.dottheta)-resX/2, max(N8.dottheta)+resX/2]);
ylim([min(N8.dotx_b)-resY/2, max(N8.dotx_b)+resY/2]);
xlabel('$\dot{\theta}$[rad/s]', 'Interpreter', 'latex');
ylabel('$\dot{x_b}$[m/s]', 'Interpreter', 'latex');
title('Estimation of $\mathcal{X}_N$ (N=8)', 'Interpreter', 'latex');