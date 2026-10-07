%% Sampling frequency exercise

close all
clear
clc

%% Motor parameters

L  = 3.57e-3;        % H
R  = 3.57;           % Ohm
Kt = 0.008863;       % Nm/A
Kb = Kt;
D  = 8.57e-7;        % Nm/(rad/s)
J  = 0.94e-6;        % kg*m^2

%% Robot geometry

gear = 9.6;
wrad = 0.03;         % wheel radius [m]

r2m = wrad/gear;     % motor rad/s -> robot m/s

%% Controller

% Kp = 0.05;
Kp = 0.013;

%% Sampling time

Ts = 0.001;          % 1 ms

%% Simulink model

model = 'motor_model_23b_nyquist';    % <-- change if your model has another name

%% Run simulation

out = sim(model, 0.6);

%% Plot

figure

plot(out.velP.Time, out.velP.Data(:,1), ...
    'LineWidth',2)

hold on

plot(out.velP.Time, out.velP.Data(:,2), ...
    'LineWidth',2)

plot(out.velP.Time, out.velP.Data(:,3), ...
    'LineWidth',2)

plot(out.velP.Time, out.velP.Data(:,4), ...
    'LineWidth',2)

legend( ...
    'Reference (m/s)', ...
    'Motor voltage / 10 (V)', ...
    'Digital velocity (m/s)', ...
    'Continuous velocity (m/s)', ...
    'Location','best')

title(sprintf('Sampling: Ts = %.3f s, Kp = %.3f',Ts,Kp))

xlabel('Time (s)')
ylabel('Velocity / Voltage')
grid on

%% Test several sampling times

% Ts_values = [0.001 0.004 0.005 0.006 0.010];
Ts_values = [0.010 0.012 0.015 0.02 0.03 0.04 0.05 0.06];

for i = 1:length(Ts_values)

    Ts = Ts_values(i);

    out = sim(model,0.6);

    figure

    plot(out.velP.Time,out.velP.Data(:,1),'LineWidth',2)
    hold on
    plot(out.velP.Time,out.velP.Data(:,2),'LineWidth',2)
    plot(out.velP.Time,out.velP.Data(:,3),'LineWidth',2)
    plot(out.velP.Time,out.velP.Data(:,4),'LineWidth',2)

    legend( ...
        'Reference (m/s)', ...
        'Motor voltage / 10 (V)', ...
        'Digital velocity (m/s)', ...
        'Continuous velocity (m/s)', ...
        'Location','best')

    title(sprintf('Sampling: Ts = %.3f s, Kp = %.3f',Ts,Kp))

    xlabel('Time (s)')
    ylabel('Velocity / Voltage')
    grid on

end