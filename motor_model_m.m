%% motor model parameters
close all
clear;
%% Motor parameters
L = 3.57e-3; % Henry
R = 3.57; % Ohm
Kt = 0.008863; % Nm/A
Kb = Kt;   % V/(rad/s)
J = 0.94e-6;  % (Kg m^2) = Nm/(rad/s^2)
D = 8.57e-7;  % Nm/(rad/s)
gear = 9.68;
wheelRadius = 0.03; % (m)
r2m = wheelRadius/gear; % (radian/s to meter/s)
% added mass
m = 0.904; % mass in kilograms
J_R = m*(wheelRadius/gear)^2;
J_tot = J + J_R;

fprintf('L  = %.4e H\n', L);
fprintf('R  = %.4f Ohm\n', R);
fprintf('Kt = Kb = %.6f Nm/A\n', Kt);
fprintf('D  = %.4e Nm/(rad/s)\n', D);
fprintf('J  = %.4e kg*m^2\n', J);

%% sim sequence
% steps 1.5V, 3V, 6V at 0.5sec interval
% format: 
% [time(sec) , Voltage left, Voltage right]
seq1 = [ 0,       0,   0; ...
         0.020, 1.5, 1.5; ...
         0.500, 3.0, 3.0; ...
         1.000, 6.0, 6.0; ...
         1.500, 0.0, 0.0; ...
         2.000, 0,   0; ...
       ];
% Three 10ms pulses of 6V with 300ms interval
% format: 
% [time(sec) , Voltage left, Voltage right]
seq2 = [0, 0, 0; ...
         0.05, 6, 6; ...
         0.06, 0, 0; ...
         0.3, 6, 6; ...
         0.31, 0, 0; ...
         0.6, 6, 6; ...
         0.61, 0, 0; ...
       ];
seq = seq1;
%% simulate (Steps)
model ='motor_model_23b';
seq = seq1;         % switch to 2-step sequence
aa = sim(model, 2); % output will be called 'aa', simulate in 2 seconds
%% simulink version of the loaded model (just debug)
info = Simulink.MDLInfo(model);
disp(info.ReleaseName); % Displays release version (e.g., 'R2023b')
%% plot result (Steps)
h = figure(10);
hold off
plot(aa.tout, aa.so.Data(:,1),'LineWidth',1.5)
hold on
plot(aa.tout, aa.so.Data(:,2),'LineWidth',1)
plot(aa.tout, aa.so.Data(:,4)/100,'LineWidth',1.5)
legend('Motor voltage (V)', 'Motor current (A)', 'Velocity (rad/s / 100)','Location','northwest')
grid on
axis([0,1.6,-0.1, 6.3])
xlabel('Time (sec)')
title('Steady state and slow transition')
grid on
saveas(h,'motor_model_steps_01.png')

%% Simulate (spikes)
seq = seq2;             % switch to spike sequence
bb = sim(model, 0.75);  % output will be in 'bb', simulate in 0.75 seconds
%% plot result (spikes)
h = figure(11);
hold off
plot(bb.tout, bb.so.Data(:,1),'LineWidth',2)
hold on
plot(bb.tout, bb.so.Data(:,2),'-x','LineWidth',2)
plot(bb.tout, bb.so.Data(:,3),'LineWidth',2)
legend('Motor voltage (V)', 'Motor current (A)', 'Velocity (m/s)','Location','best')
grid on
axis([0.048,0.063,-0.75, 7])
xlabel('Time (sec)')
title('Fast transition (first spike)')
grid on
saveas(h,'motor_model_spikes_01.png')
%
%

%% Transfer function
Gs = tf(Kt, [J*L (D*L + J*R) (D*R + Kt*Kb)]);

[num, den] = pade(0.01, 2);   % 10 ms delay, 2nd-order Pade
Gd = tf(num, den);


%% Frequency vector

w = logspace(-1, 5, 2000);   % rad/s


%% Extract Bode data

[mag1, phase1] = bode(Gs, w);
[mag2, phase2] = bode(Gs*Gd, w);

mag1 = squeeze(mag1);
mag2 = squeeze(mag2);

phase1 = squeeze(phase1);
phase2 = squeeze(phase2);


%% Convert magnitude to dB

mag1_dB = 20*log10(mag1);
mag2_dB = 20*log10(mag2);


%% Unwrap phase

phase1 = rad2deg(unwrap(deg2rad(phase1)));
phase2 = rad2deg(unwrap(deg2rad(phase2)));


%% Force phase to start around 0 deg instead of 360 deg

if phase1(1) > 180
    phase1 = phase1 - 360;
end

if phase2(1) > 180
    phase2 = phase2 - 360;
end


%% Plot

figure(15);
clf;


% Magnitude
subplot(2,1,1)

semilogx(w, mag1_dB, 'LineWidth', 1.5);
hold on
semilogx(w, mag2_dB, 'LineWidth', 1.5);

grid on

ylabel('Magnitude [dB]')
title('Bode Plot')

legend('No delay', '10 ms delay', ...
       'Location', 'best')


% Phase
subplot(2,1,2)

semilogx(w, phase1, 'LineWidth', 1.5);
hold on
semilogx(w, phase2, 'LineWidth', 1.5);

grid on

xlabel('Frequency [rad/s]')
ylabel('Phase [deg]')

legend('No delay', '10 ms delay', ...
       'Location', 'best')

%% Motor Model + Controller Simulation

time_delay = 0.01;

K_P = 0.013; % Eu: 0.013 ; 0.011 para ficar com wc < 30

tau_I = 0.04; % Eu: 0.04 

sim_time = 0.5;

sat_upper_limit = 8;
sat_lower_limit = -8;

step_time = 0.1;
final_step_value = 100;

cc = sim("motor_model_23b_controller.slx", sim_time);

figure;

% System response
plot(cc.tout, cc.so.Data(:,4), 'LineWidth', 2);
hold on;

% Reference step
ref = final_step_value * (cc.tout >= step_time);
plot(cc.tout, ref, '--', 'LineWidth', 2);

grid on;
xlabel('Time [s]');
ylabel('Motor velocity [rad/s]');
legend('Motor velocity', 'Reference');

% Bode diagram interpretation

s = tf('s');

C1 = 0.178 * (1 + 1/(0.087*s));
C2 = K_P   * (1 + 1/(tau_I*s));

G = Gs * Gd;

L1 = C1 * G;
L2 = C2 * G;

figure;
margin(L1);
grid on;
title('PI - Bode design');

figure;
margin(L2);
grid on;
title('Open-Loop PI Controller Bode');
% 
% isstable(feedback(L1,1))
% isstable(feedback(L2,1))
