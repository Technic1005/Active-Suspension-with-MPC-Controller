clear;clc;close all
load('OutputMPC.mat')
%%
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
div = 15;
d1 = [zeros(1,T/div) 0.1*ones(1,1*T/div) zeros(1,8*T/div) 0.1*ones(1,5*T/div)];
%%
figure(1)
subplot(3,1,1)
stairs(timestamp(1:end-1), y(1,:))
subplot(3,1,2)
hold on
stairs(timestamp(1:end-1), d1*0.5)
stairs(timestamp(1:end-1), yref(3,:))
stairs(timestamp(1:end-1), y(3,:))
subplot(3,1,3)
hold on
stairs(timestamp(1:end-1), d1*0.5)
stairs(timestamp(1:end-1), yref(4,:))
stairs(timestamp(1:end-1), y(4,:))
%%
figure()
hold on
stairs(timestamp(1:end-1), u_rec(1,:))
stairs(timestamp(1:end-1), ur_plot(1,:))
%%
ddotx_b = timeseries(y(1,:), timestamp(1:end-1));
dotx_b = timeseries(xe(1,:), timestamp(1:end));
