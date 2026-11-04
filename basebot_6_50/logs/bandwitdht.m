clear; clc; close all;

%% MOTOR PARAMETERS

L  = 3.57e-3;
R  = 3.57;
Kt = 0.008863;
Kb = Kt;
D  = 8.57e-7;

J = 0.94e-6 + 8.683e-6;

%% MOTOR TRANSFER FUNCTION

s = tf('s');

% Motor angular velocity [rad/s] / Voltage [V]
G = Kt / ((J*s + D)*(L*s + R) + Kt*Kb);

%% PI CONTROLLERS

names = {
    'Original'
    'Minimum RMSE'
    'Best compromise'
    'Minimum settling time'
    'Previous compromise'
};

Kp = [
    0.080
    0.036
    0.040
    0.040
    0.040
];

tauI = [
    0.400
    0.180
    0.160
    0.140
    0.180
];

Ki = Kp ./ tauI;

%% CLOSED-LOOP BANDWIDTH

n = length(Kp);

BW = zeros(n,1);
T = cell(n,1);

for i = 1:n

    C = Kp(i) + Ki(i)/s;

    % Closed-loop transfer function
    T{i} = feedback(C*G,1);

    % -3 dB bandwidth
    BW(i) = bandwidth(T{i});

end

%% RESULTS TABLE

Results = table( ...
    names, Kp, tauI, Ki, BW, BW/(2*pi), ...
    'VariableNames', { ...
    'Controller', 'Kp', 'tauI_s', 'Ki', ...
    'Bandwidth_rad_s', 'Bandwidth_Hz'});

disp('===== WHEEL SPEED CLOSED-LOOP BANDWIDTH =====')
disp(Results)

%% COMPARISON WITH ORIGINAL

fprintf('\n===== RELATIVE BANDWIDTH =====\n');

for i = 2:n

    fprintf('%s: %+.2f %% vs original\n', ...
        names{i}, 100*(BW(i)/BW(1)-1));

end

%% CASCADE CONTROL CHECK

% Desired: inner loop at least 3 times faster than outer loop

fprintf('\n===== MAXIMUM OUTER LOOP BANDWIDTH (3:1) =====\n');

for i = 1:n

    fprintf('%s: %.3f rad/s\n', ...
        names{i}, BW(i)/3);

end

%% BODE PLOT

figure;
hold on;
grid on;

for i = 1:n

    [mag,~,w] = bode(T{i});

    semilogx(squeeze(w), ...
        20*log10(squeeze(mag)), ...
        'LineWidth',1.5, ...
        'DisplayName',names{i});

end

yline(-3.0103,'k--','-3 dB');

xlabel('Frequency [rad/s]');
ylabel('Magnitude [dB]');
title('Wheel Speed Closed-Loop Frequency Response');
legend('Location','best');