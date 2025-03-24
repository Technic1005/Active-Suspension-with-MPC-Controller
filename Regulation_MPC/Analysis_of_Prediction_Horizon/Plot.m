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
hold on
index = 1;
Timestamp = N2.step * 0.01;
stairs(Timestamp, N2.x_original(index,:))
stairs(Timestamp, N2.x(index,:))
stairs(Timestamp, N5.x(index,:))
stairs(Timestamp, N8.x(index,:))
stairs(Timestamp, N15.x(index,:))
legend('No Control','N=2','N=5','N=8','N=15', 'Interpreter', 'latex')
xlabel('Time[s]', 'Interpreter', 'latex')
ylabel(ylabels{1}, 'Interpreter', 'latex')
title(titles(1), 'Interpreter', 'latex')
xlim([0,2])

%% Cmp All States
figure(2)
Timestamp = N2.step * 0.01;
index_map = [1,2,3,4,5,6,7,8];

for i = 1:length(index_map)
    subplot(4,2,i)
    hold on
    index = index_map(i);
    stairs(Timestamp, N2.x_original(index,:))
    stairs(Timestamp, N2.x(index,:))
    stairs(Timestamp, N5.x(index,:))
    stairs(Timestamp, N8.x(index,:))
    stairs(Timestamp, N15.x(index,:))
    legend('No Control','N=2','N=5','N=8','N=15', 'Interpreter', 'latex')
    xlabel('Time[s]', 'Interpreter', 'latex')
    ylabel(ylabels{index}, 'Interpreter', 'latex')
    title(titles(index), 'Interpreter', 'latex')
end