clear;clc;close all
load('OutputMPC.mat')
%% Load Data
titles = {
    'State1: Sprung Mass Velocity';
    'State2: Pitch Rate';
    'State3: Unsprung Mass Velocity (Front)';
    'State4: Unsprung Mass Velocity (Rear)';
    'State5: Suspension Travel (Front)';
    'State6: Suspension Travel (Rear)';
    'State7: Tire Deformation (Front)';
    'State8: Tire Deformation (Rear)'
    };
ylabels = {
    '$\dot{x_b}$[m/s]';
    '$\dot{\theta}$[rad/s]';
    '$\dot{x_{w1}}$[m/s]';
    '$\dot{x_{w2}}$[m/s]';
    '$x_{sus1}$[m]';
    '$x_{sus2}$[m]';
    '$x_{t1}$[m]';
    '$x_{t2}$[m]'
    };
timestamp = 0:0.01:15;
%% Plot Output States
figure(1)
set(gcf, 'Position', [100, 100, 800, 600]) % Figure Size

% Colormap
colors = lines(4); 

% Output State 1
subplot(3,1,1)
stairs(timestamp(1:end-1), y(1,:), 'Color', colors(1,:), 'LineWidth', 1.5)
grid on
xlabel('Time [s]', 'Interpreter', 'latex', 'FontSize', 12)
ylabel(ylabels{2}, 'Interpreter', 'latex', 'FontSize', 12)
title('Output State: Pitch Rate', 'Interpreter', 'latex', 'FontSize', 14)

% Output State 2
subplot(3,1,2)
hold on
% Disturbances
stairs(timestamp(1:end-1), d1*10, '--', 'Color', colors(2,:), 'LineWidth', 1.5)
stairs(timestamp(1:end-1), d2*10, '--', 'Color', colors(3,:), 'LineWidth', 1.5)
stairs(timestamp(1:end-1), yref(2,:), '-', 'Color', colors(4,:), 'LineWidth', 2)
stairs(timestamp(1:end-1), y(2,:), '-', 'Color', colors(1,:), 'LineWidth', 2)
grid on
xlabel('Time [s]', 'Interpreter', 'latex', 'FontSize', 12)
ylabel(ylabels{5}, 'Interpreter', 'latex', 'FontSize', 12)
legend({'Disturbance 1', 'Disturbance 2', 'Reference', '$x_{sus1}$'}, ...
       'Interpreter', 'latex', 'FontSize', 10, 'Location', 'best')
title('Output State: Front Suspension Height', 'Interpreter', 'latex', 'FontSize', 14)

% Output State 3
subplot(3,1,3)
hold on
stairs(timestamp(1:end-1), d1*10, '--', 'Color', colors(2,:), 'LineWidth', 1.5)
stairs(timestamp(1:end-1), d2*10, '--', 'Color', colors(3,:), 'LineWidth', 1.5)
stairs(timestamp(1:end-1), yref(3,:), '-', 'Color', colors(4,:), 'LineWidth', 2)
stairs(timestamp(1:end-1), y(3,:), '-', 'Color', colors(1,:), 'LineWidth', 2)
grid on
xlabel('Time [s]', 'Interpreter', 'latex', 'FontSize', 12)
ylabel(ylabels{6}, 'Interpreter', 'latex', 'FontSize', 12)
legend({'Disturbance 1', 'Disturbance 2', 'Reference', '$x_{sus2}$'}, ...
       'Interpreter', 'latex', 'FontSize', 10, 'Location', 'best')
title('Output State: Rear Suspension Height', 'Interpreter', 'latex', 'FontSize', 14)

%% Plot Disturbances
figure(2)
set(gcf, 'Position', [100, 100, 800, 500]) 

colors = lines(2);

% disturbances 1
subplot(2,1,1)
hold on
stairs(timestamp(1:end-1), d1 * 5, '--', 'Color', colors(1,:), 'LineWidth', 1.5) % d1
stairs(timestamp, xehat(9,:) * 5, '-', 'Color', colors(2,:), 'LineWidth', 2) % estimated d1
grid on
xlabel('Time [s]', 'Interpreter', 'latex', 'FontSize', 12)
ylabel('Disturbance 1', 'Interpreter', 'latex', 'FontSize', 12)
legend({'$d_1$', '$d_1$ estimated'}, 'Interpreter', 'latex', 'FontSize', 10, 'Location', 'southeast')
title('Disturbance 1 Estimation', 'Interpreter', 'latex', 'FontSize', 14)

% disturbances 2
subplot(2,1,2)
hold on
stairs(timestamp(1:end-1), d2 * 5, '--', 'Color', colors(1,:), 'LineWidth', 1.5) % d2
stairs(timestamp, xehat(10,:) * 5, '-', 'Color', colors(2,:), 'LineWidth', 2) % estimated d2
grid on
xlabel('Time [s]', 'Interpreter', 'latex', 'FontSize', 12)
ylabel('Disturbance 2', 'Interpreter', 'latex', 'FontSize', 12)
legend({'$d_2$', '$d_2$ estimated'}, 'Interpreter', 'latex', 'FontSize', 10, 'Location', 'southeast')
title('Disturbance 2 Estimation', 'Interpreter', 'latex', 'FontSize', 14)


%% State and Control References
figure(3)
set(gcf, 'Position', [100, 100, 800, 500])
colors = lines(8);

% State References
subplot(2,1,1)
hold on
for i = 1:8
    stairs(timestamp(1:end-1), xr_plot(i,:), 'Color', colors(i,:), 'LineWidth', 1.5)
end
grid on
legend(ylabels, 'Interpreter', 'latex', 'FontSize', 10, 'Location', 'northwest')
xlabel('Time [s]', 'Interpreter', 'latex', 'FontSize', 12)
ylabel('States References', 'Interpreter', 'latex', 'FontSize', 12)
title('State Reference Trajectories', 'Interpreter', 'latex', 'FontSize', 14)

% Control References
subplot(2,1,2)
hold on
plot(timestamp(1:end-1), ur_plot(1,:), '-', 'Color', colors(1,:), 'LineWidth', 2)
plot(timestamp(1:end-1), ur_plot(2,:), '-', 'Color', colors(2,:), 'LineWidth', 2)
grid on
xlabel('Time [s]', 'Interpreter', 'latex', 'FontSize', 12)
ylabel('Inputs References', 'Interpreter', 'latex', 'FontSize', 12)
legend({'$F_1$', '$F_2$'}, 'Interpreter', 'latex', 'FontSize', 10, 'Location', 'northwest')
title('Input Reference Trajectories', 'Interpreter', 'latex', 'FontSize', 14)


