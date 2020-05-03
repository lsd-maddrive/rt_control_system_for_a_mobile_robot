clc; clear; close all;

% Initial data that from data sheet:
U_rated = 9;                            % V
U_source = 14.8;                        % V
k_p = 1;                                % transfer koef
M_rated = 0.03;                         % kg*m
I_rated = 1.2;                          % A
w_rated = 100;                          % rpm
% Data calculation:
M_rated = M_rated*9.8                   % N*m
w_rated = w_rated*6.28/60               % rad/sec
Cm = M_rated / I_rated                  % N*m/A
Ce = Cm                                 % N*m/A
R_ya = (U_rated - Ce*w_rated)/I_rated	% Om
% Choose using experiments:
M_st = M_rated*0.22                     % Because of max speed value
k_vt = 0.0001                           % Because of max speed value
J = 0.00075                             % Because of time of transition process
L_ya = 0.01                             % ?


% Constants
SIM_TIME = 1.0;
STEP_SIZE = 0.001;
RPM_TO_TPS = 200/60;            % rotation per minutes to ticks per second
TICK_TO_METER = 0.0005167;      % calibration constant
METER_TO_TICK = 1/0.0005167;    % calibration constant
DESIRED_SPEED = 0.15
kp_real = 0.025                 % on real robot
ki_real = 0.25                  % on real robot
kd_real = 0                     % on real robot
kp = 0.0850                     % optimized
ki = 1.977%8.42                 % optimized
kd = 0                          % optimized
fuzzy_regulator = readfis('fuzzy_regulator.fis');

% Real PI-regulator
fname = 'test_pid_result_3.json'; 
fid = fopen(fname); 
raw = fread(fid,inf); 
str = char(raw'); 
fclose(fid); 
real_data = jsondecode(str);

REGULATORS = ["fuzzy modeling", "optimazed PI modeling"];
%REGULATORS = ["PI on robot", "PI modeling"];
%REGULATORS = ["raw"];
result_plot = figure;
for REGULATOR = REGULATORS
    if strcmp(REGULATOR, 'raw')
        REG_TYPE = 1;
        sim('dc_motor')
    elseif strcmp(REGULATOR, 'PI modeling')
        REG_TYPE = 2;
        sim('dc_motor')
    elseif strcmp(REGULATOR, 'optimazed PI modeling')
        REG_TYPE = 3;
        sim('dc_motor')
    elseif strcmp(REGULATOR, 'fuzzy modeling')
        REG_TYPE = 4;
        sim('dc_motor')
        subplot(2, 3, 4)
        hold on
        plot(fuzzy_data.time, fuzzy_data.signals.values)
    elseif strcmp(REGULATOR, 'PI on robot')
        speed.time = real_data.linearSpeed.time(1:end/6) - 0.12;
        speed.signals.values = real_data.linearSpeed.data(1:end/6);
        encoder.time = real_data.leftEncoderSpeed.time(1:end/6) - 0.12;
        encoder.signals.values = real_data.leftEncoderSpeed.data(1:end/6);
        voltage.time = real_data.leftMotorPwm.time(1:end/6) - 0.12;
        voltage.signals.values = real_data.leftMotorPwm.data(1:end/6);
        fuzzy_data.time = 0;
        fuzzy_data.signals.values = 0;
        speed_of_revolutions.time = 0;
        speed_of_revolutions.signals.values = 0;
    else
        continue
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
    
    subplot(2, 3, 5)
    hold on
    plot(speed_of_revolutions.time, speed_of_revolutions.signals.values)
end

% Add info to plots
subplot(2, 3, 1)
hold on
%plot([0 speed.time(end)], [DESIRED_SPEED DESIRED_SPEED])
xlabel('time, sec')
title('speed, m/sec')

legend(REGULATORS)
grid on

subplot(2, 3, 2)
hold on
title('encoder, ticks')
xlabel('time, sec')
legend(REGULATORS)
grid on

subplot(2, 3, 3)
hold on
title('PWM, duty cycle')
xlabel('time, sec')
legend(REGULATORS)
grid on

subplot(2, 3, 4)
legend('error', 'integral', 'out')

subplot(2, 3, 5)
title('speed of revolution, rpm')
legend(REGULATORS)