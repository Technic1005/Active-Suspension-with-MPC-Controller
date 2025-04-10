clear;clc;close all
%% Load Data
N2 = load("Neq2.mat");
N5 = load("Neq5.mat");
N8 = load("Neq8.mat");
N15 = load("Neq15.mat");

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
%% Cmp Prediction Horizon Figure
figure(1)
index = 1;
set(gcf, 'Position', [100, 100, 800, 500]) % Window Size
hold on

% Colormap
colors = lines(5);
line_styles = {'-', '-', '-', '-', '-'};

% Main figure
Timestamp = N2.step * 0.01;
%stairs(Timestamp, N2.x_original(index,:), 'LineStyle', line_styles{1}, 'Color', colors(1,:), 'LineWidth', 1.5)
stairs(Timestamp, N2.x(index,:), 'LineStyle', line_styles{2}, 'Color', colors(2,:), 'LineWidth', 1.5)
stairs(Timestamp, N5.x(index,:), 'LineStyle', line_styles{3}, 'Color', colors(3,:), 'LineWidth', 1.5)
stairs(Timestamp, N8.x(index,:), 'LineStyle', line_styles{4}, 'Color', colors(4,:), 'LineWidth', 1.5)
stairs(Timestamp, N15.x(index,:), 'LineStyle', line_styles{5}, 'Color', colors(5,:), 'LineWidth', 2)

% Legend
legend({'N=2', 'N=5', 'N=8', 'N=15'}, ...
       'Interpreter', 'latex', 'FontSize', 10, 'Location', 'northwest')
xlabel('Time [s]', 'Interpreter', 'latex', 'FontSize', 12)
ylabel(ylabels{index}, 'Interpreter', 'latex', 'FontSize', 12)

% Title
title(titles(index), 'Interpreter', 'latex', 'FontSize', 14)

xlim([0, 1.2])
ylim([-0.08, 0.1])
grid on
