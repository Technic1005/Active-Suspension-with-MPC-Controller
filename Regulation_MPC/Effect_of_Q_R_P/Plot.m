clear;clc;close all
%% Load Data
Base = load("baseline.mat");
Q01 = load("Q0.1.mat");
Q10 = load("Q10.mat");
Q100 = load("Q100.mat");
R001 = load('R0.01.mat');
R01 = load('R0.1.mat');
R10 = load('R10.mat');

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
Timestamp = Base.step * 0.01;
%% Q CMP
figure()
set(gcf, 'Position', [100, 100, 900, 600]) % Figure size

% colormap
colors = lines(4);
line_styles = {'-', '--', '-.', ':'};

index_map = [1,2,5,6];
for i = 1:length(index_map)
    subplot(2,2,i)
    hold on
    
    % Different Q values
    plot(Timestamp, Q01.x(index_map(i),:), 'LineStyle', line_styles{1}, 'Color', colors(1,:), 'LineWidth', 1.5)
    plot(Timestamp, Base.x(index_map(i),:), 'LineStyle', line_styles{2}, 'Color', colors(2,:), 'LineWidth', 1.5)
    plot(Timestamp, Q10.x(index_map(i),:), 'LineStyle', line_styles{3}, 'Color', colors(3,:), 'LineWidth', 1.5)
    plot(Timestamp, Q100.x(index_map(i),:), 'LineStyle', line_styles{4}, 'Color', colors(4,:), 'LineWidth', 2)
    
    % lims
    xlim([0, 2])
    % ylim([-0.5, 0.5]) % 可根据数据调整

    % Labels
    xlabel('Time [s]', 'Interpreter', 'latex', 'FontSize', 12)
    ylabel(ylabels{index_map(i)}, 'Interpreter', 'latex', 'FontSize', 12)

    % Title
    title(['State: ', ylabels{index_map(i)}], 'Interpreter', 'latex', 'FontSize', 14)

    grid on
end

% Legend
legend({'$0.1Q$', '$Q$', '$10Q$', '$100Q$'}, ...
       'Interpreter', 'latex', 'FontSize', 10, 'Location', 'bestoutside')

%% R CMP
figure()
set(gcf, 'Position', [100, 100, 900, 600]) % Figure size

% colormap
colors = lines(4);
line_styles = {'-', '--', '-.', ':'};

index_map = [1,2,5,6];
for i = 1:length(index_map)
    subplot(2,2,i)
    hold on
    
    % Different R values
    plot(Timestamp, R001.x(index_map(i),:), 'LineStyle', line_styles{1}, 'Color', colors(1,:), 'LineWidth', 1.5)
    plot(Timestamp, R01.x(index_map(i),:), 'LineStyle', line_styles{2}, 'Color', colors(2,:), 'LineWidth', 1.5)
    plot(Timestamp, Base.x(index_map(i),:), 'LineStyle', line_styles{3}, 'Color', colors(3,:), 'LineWidth', 1.5)
    plot(Timestamp, R10.x(index_map(i),:), 'LineStyle', line_styles{4}, 'Color', colors(4,:), 'LineWidth', 2)
    
    % lim
    xlim([0, 2])
    % ylim([-0.5, 0.5]) % 可根据数据调整

    % Labels
    xlabel('Time [s]', 'Interpreter', 'latex', 'FontSize', 12)
    ylabel(ylabels{index_map(i)}, 'Interpreter', 'latex', 'FontSize', 12)

    % Title
    title(['State: ', ylabels{index_map(i)}], 'Interpreter', 'latex', 'FontSize', 14)

    grid on
end

% Legends
legend({'$0.01R$', '$0.1R$', '$R$', '$10R$'}, ...
       'Interpreter', 'latex', 'FontSize', 10, 'Location', 'bestoutside')

%% R Control Input CMP
figure()
set(gcf, 'Position', [100, 100, 800, 500]) % Figure size

% colormap
colors = lines(4);
line_styles = {'-', '--', '-.', ':'};

subplot(2,1,1)
hold on
plot(Timestamp(1:end-1), R001.u_rec(1,:), 'LineStyle', line_styles{1}, 'Color', colors(1,:), 'LineWidth', 1.5)
plot(Timestamp(1:end-1), R01.u_rec(1,:), 'LineStyle', line_styles{2}, 'Color', colors(2,:), 'LineWidth', 1.5)
plot(Timestamp(1:end-1), Base.u_rec(1,:), 'LineStyle', line_styles{3}, 'Color', colors(3,:), 'LineWidth', 1.5)
plot(Timestamp(1:end-1), R10.u_rec(1,:), 'LineStyle', line_styles{4}, 'Color', colors(4,:), 'LineWidth', 2)

xlim([0, 1])

% Labels and titles
xlabel('Time [s]', 'Interpreter', 'latex', 'FontSize', 12)
ylabel('$F_1$ [N]', 'Interpreter', 'latex', 'FontSize', 12)
title('Control Input: $F_1$', 'Interpreter', 'latex', 'FontSize', 14)

grid on

subplot(2,1,2)
hold on
plot(Timestamp(1:end-1), R001.u_rec(2,:), 'LineStyle', line_styles{1}, 'Color', colors(1,:), 'LineWidth', 1.5)
plot(Timestamp(1:end-1), R01.u_rec(2,:), 'LineStyle', line_styles{2}, 'Color', colors(2,:), 'LineWidth', 1.5)
plot(Timestamp(1:end-1), Base.u_rec(2,:), 'LineStyle', line_styles{3}, 'Color', colors(3,:), 'LineWidth', 1.5)
plot(Timestamp(1:end-1), R10.u_rec(2,:), 'LineStyle', line_styles{4}, 'Color', colors(4,:), 'LineWidth', 2)

xlim([0, 1])

xlabel('Time [s]', 'Interpreter', 'latex', 'FontSize', 12)
ylabel('$F_2$ [N]', 'Interpreter', 'latex', 'FontSize', 12)
title('Control Input: $F_2$', 'Interpreter', 'latex', 'FontSize', 14)

grid on

% Legends
legend({'$0.01R$', '$0.1R$', '$R$', '$10R$'}, ...
       'Interpreter', 'latex', 'FontSize', 10, 'Location', 'bestoutside')
