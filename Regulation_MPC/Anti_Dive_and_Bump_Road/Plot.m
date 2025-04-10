clear;clc;close all
%% Load Data
Result = load("result.mat");

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
Timestamp = Result.step * 0.01;

%% With and Without controller CMP: States
figure()
set(gcf, 'Position', [100, 100, 900, 600]) % figure size

% colormap
colors = lines(2);
line_styles = {'-', '-'};

index_map = [1,2,5,6];
for i = 1:length(index_map)
    subplot(2,2,i)
    hold on

    % State trajectory
    plot(Timestamp, Result.x_original(index_map(i),:), 'LineStyle', line_styles{1}, 'Color', colors(1,:), 'LineWidth', 1.5)
    plot(Timestamp, Result.x(index_map(i),:), 'LineStyle', line_styles{2}, 'Color', colors(2,:), 'LineWidth', 1.5)
    
    % Labels
    xlabel('Time [s]', 'Interpreter', 'latex', 'FontSize', 12)
    ylabel(ylabels{index_map(i)}, 'Interpreter', 'latex', 'FontSize', 12)

    grid on

    % Title
    title(['State: ', ylabels{index_map(i)}], 'Interpreter', 'latex', 'FontSize', 14)
end

% Legend
legend({'Without Control', 'MPC'}, 'Interpreter', 'latex', 'FontSize', 12, 'Location', 'bestoutside')

%% With and Without controller CMP: Actuators
figure()
set(gcf, 'Position', [100, 100, 800, 400]) % figure size

hold on
plot(Timestamp(1:end-1), Result.u_rec(1,:), 'LineStyle', '-', 'Color', 'b', 'LineWidth', 1.5)
plot(Timestamp(1:end-1), Result.u_rec(2,:), 'LineStyle', '-', 'Color', 'r', 'LineWidth', 1.5)

% lim
ylim([-2000, 2000])

% labels
xlabel('Time [s]', 'Interpreter', 'latex', 'FontSize', 12)
ylabel('Actuator Force [N]', 'Interpreter', 'latex', 'FontSize', 12)

grid on

% Title
title('Actuator Forces', 'Interpreter', 'latex', 'FontSize', 14)

% Legend
legend({'Front', 'Rear'}, 'Interpreter', 'latex', 'FontSize', 12, 'Location', 'best')
