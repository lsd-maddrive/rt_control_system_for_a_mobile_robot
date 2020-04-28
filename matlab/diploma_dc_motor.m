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
SIM_TIME = 1.0;
RPM_TO_TPS = 200/60;            % rotation per minutes to ticks per second
TICK_TO_METER = 0.0005167;      % calibration constant
METER_TO_TICK = 1/0.0005167;    % calibration constant
DESIRED_SPEED = 0.15
kp = 0.025
ki = 0.25
sim('dc_motor')

figure
plot(speed_of_revolutions.time, speed_of_revolutions.signals.values)
title('speed of revolution, rpm')
grid()

figure
subplot(1, 3, 1)
plot(speed.time, speed.signals.values)
title('speed, m/sec')
grid()

subplot(1, 3, 2)
plot(encoder.time, encoder.signals.values)
title('encoder, ticks')
grid()

subplot(1, 3, 3)
plot(voltage.time, voltage.signals.values)
title('PWM, duty cycle')
grid()