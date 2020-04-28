clc; clear; close all;

% Initial data that from data sheet:
U = 9                           % V
k_p = 1                         % transfer koef
M_rated = 0.03;                 % kg*m
I_rated = 1.2;                  % A
w_rated = 100;                  % rpm
% Data calculation:
M_rated = M_rated*9.8;          % N*m
w_rated = w_rated*6.28/60       % rad/sec
Cm = M_rated / I_rated          % N*m/A
Ce = Cm                         % N*m/A
R_ya = (U - Ce*w_rated)/I_rated % Om
% Choose using experiments:
M_st = M_rated*1.5              % Because of max speed value
k_vt = 0.05                     % Because of max speed value
J = 0.00075                     % Because of time of transition process
L_ya = 0.01                     % ?


% Constants
SIM_TIME = .1;
STEP_SIZE = 0.001;
RPM_TO_TPS = 200/60;            % rotation per minutes to ticks per second
TICK_TO_METER = 0.0005167;      % calibration constant
METER_TO_TICK = 1/0.0005167;    % calibration constant
DESIRED_SPEED = 0.15
% kp = 0.025                    % default
% ki = 0.25                     % default
% kd = 0                        % default
kp = 0.0850                     % optimized
ki = 8.42                       % optimized
kd = 0.0001340                  % optimized
fuzzy_regulator = readfis('fuzzy_regulator.fis');

% FIS = mamfis;
% FIS = addInput(FIS,[-500 500],'Name','E');
% FIS = addMF(FIS,'E','trimf',[-1000 -10 0],'Name','Negative');
% FIS = addMF(FIS,'E','trimf',[-10 0 10],'Name','Zero');
% FIS = addMF(FIS,'E','trimf',[0 10 20],'Name','Positive');


for i = 0:1
    if i == 0
        FZ_INSTEAD_PID = 1;
        sim('dc_motor')
        result_plot = figure;
    elseif i == 1
        FZ_INSTEAD_PID = 0;
        sim('dc_motor')
    end
    subplot(2, 3, 1)
    hold on
    plot(speed.time, speed.signals.values)
    
    subplot(2, 3, 2)
    hold on
    plot(encoder.time, encoder.signals.values)

    subplot(2, 3, 3)
    hold on
    plot(voltage.time, voltage.signals.values)
    
    subplot(2, 3, 4)
    hold on
    plot(fuzzy_data.time, fuzzy_data.signals.values)
    
    subplot(2, 3, 5)
    hold on
    plot(speed_of_revolutions.time, speed_of_revolutions.signals.values)
end

% Add info to plots
subplot(2, 3, 1)
hold on
plot([0 speed.time(end)], [DESIRED_SPEED DESIRED_SPEED])
title('speed, m/sec')
legend('fuzzy', 'pid')
grid on

subplot(2, 3, 2)
hold on
title('encoder, ticks')
legend('fuzzy', 'pid')
grid on

subplot(2, 3, 3)
hold on
title('PWM, duty cycle')
legend('fuzzy', 'pid')
grid on

subplot(2, 3, 4)
legend('difference', 'derivative', 'integral', 'out')

subplot(2, 3, 5)
title('speed of revolution, rpm')
legend('fuzzy', 'pid')