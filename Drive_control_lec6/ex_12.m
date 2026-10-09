%% Exercise 1b - Velocity and turnrate control

close all
clear
clc

%% Motor parameters

L  = 3.57e-3;        % H
R  = 3.57;           % Ohm
Kt = 0.008863;       % Nm/A
Kb = Kt;
D  = 8.57e-7;        % Nm/(rad/s)
J_mot  = 0.94e-6;        % kg*m^2
J = J_mot + 8.683e-6;

%% Robot geometry

gear = 9.6;
wrad = 0.03;         % wheel radius [m]
r2m = wrad/gear;

B = 0.147;            % wheel base [m]

%% Reference

Tsim = 20; % s

% Fixed Single Step

% velocity_ref = 0.6; % m/s
% turnrate_ref = 1.0; % rad/s

% Slalom Trajectory

t_ref = (0:0.01:Tsim)';

velocity_ref = 0.4 + 0.15*sin(2*pi*t_ref/10);
turnrate_ref = 0.7*sin(2*pi*t_ref/8);

ramp = min(t_ref/1, 1);

velocity_ref = velocity_ref .* ramp;
turnrate_ref = turnrate_ref .* ramp;

v_input = [t_ref velocity_ref];
w_input = [t_ref turnrate_ref];

% Infinite (8) Trajectory

% t_ref = (0:0.01:Tsim)';
% 
% T_turn = 2;
% T = Tsim - T_turn;
% A = 1.5;
% B = 0.75;
% Omega = 2*pi/T;
% 
% velocity_ref = zeros(size(t_ref));
% turnrate_ref = zeros(size(t_ref));
% 
% % Initial rotation of 45 degrees
% idx1 = t_ref <= T_turn;
% turnrate_ref(idx1) = (pi/4)/T_turn * ...
%     (1 - cos(2*pi*t_ref(idx1)/T_turn));
% 
% % Figure-eight trajectory
% idx2 = t_ref > T_turn;
% tau = t_ref(idx2) - T_turn;
% 
% dx = A*Omega*cos(Omega*tau);
% dy = 2*B*Omega*cos(2*Omega*tau);
% 
% ddx = -A*Omega^2*sin(Omega*tau);
% ddy = -4*B*Omega^2*sin(2*Omega*tau);
% 
% velocity_ref(idx2) = sqrt(dx.^2 + dy.^2);
% turnrate_ref(idx2) = (dx.*ddy - dy.*ddx)./(dx.^2 + dy.^2);
% 
% v_input = [t_ref velocity_ref];
% w_input = [t_ref turnrate_ref];


%% Controller

Kp = 0.04;
tau_I = 0.4;
Ki = Kp/tau_I;

%% Sampling time

Ts = 0.008;

%% Simulink model

model = 'motor_model_23b_lec6';

out = sim(model,Tsim);

%% Exercise 1b plot

t = out.robot.Time;
data = out.robot.Data;

% Order of signals in the Mux, as in the lecture slide:
vRef        = data(:,1);
turnrateRef = data(:,2);
distance    = data(:,3);
velocity    = data(:,4);
heading     = data(:,5);
turnrate    = data(:,6);

figure

plot(t,vRef,'LineWidth',1.5)
hold on
plot(t,turnrateRef,'LineWidth',1.5)
plot(t,distance,'LineWidth',1.5)
plot(t,velocity,'LineWidth',1.5)
plot(t,heading,'LineWidth',1.5)
plot(t,turnrate,'LineWidth',1.5)

grid on
xlabel('Time (s)')
ylabel('Value')

legend('Velocity Step (m/s)', ...   
       'Turnrate Step (rad/s)', ...
       'Robot model distance (m)', ...
       'Robot model vel (m/s)', ...
       'Robot model heading (rad)', ...
       'Robot model turnrate (rad/s)', ...
       'Location','best')

title('Exercise 1b - Velocity and turnrate control')

% Motor Inputs

t_ML = out.velP.Time;
data_ML = out.velP.Data;

t_MR = out.velPR.Time;
data_MR = out.velPR.Data;

input_ML = data_ML(:,5);
input_MR = data_MR(:,5);

figure;
plot(t_ML,input_ML,'LineWidth',1.5)
hold on
plot(t_MR,input_MR,'LineWidth',1.5)
grid on
xlabel('Time (s)')
ylabel('Motor input')
legend('Left motor','Right motor','Location','best')
title('Actuator Inputs [V]');

% Robot trajectory
x = cumtrapz(t, velocity .* cos(heading));
y = cumtrapz(t, velocity .* sin(heading));

figure
plot(x, y, 'b', 'LineWidth', 1.5)
hold on
plot(0, 0, 'go', 'MarkerFaceColor', 'g')
plot(x(end), y(end), 'ro', 'MarkerFaceColor', 'r')

grid on
axis equal
xlabel('X (m)')
ylabel('Y (m)')
legend('Robot trajectory', 'Initial position', 'Final position', ...
       'Location', 'best')
title('Robot trajectory in the XY plane')