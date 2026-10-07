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
Kp = 0.008;

%% Sampling time

Ts = 0.001;          % 1 ms



%% Robot parameters
B = 0.15;          % wheel base [m]

%% Run Simulink
model = 'motor_model_23b_lec6';     % muda se o teu modelo tiver outro nome
out = sim(model, 1.5);

%% Motor velocities
% velP / velPR:
% col 1 = reference
% col 2 = motor voltage/10
% col 3 = wheel velocity [m/s]

t  = out.velP.Time;

vL = out.velP.Data(:,3);
vR = out.velPR.Data(:,3);

%% Robot velocity and turn rate
v = (vL + vR)/2;
w = (vR - vL)/B;

%% Pose
theta = cumtrapz(t,w);

xdot = v .* cos(theta);
ydot = v .* sin(theta);

x = cumtrapz(t,xdot);
y = cumtrapz(t,ydot);

distance = cumtrapz(t,v);

%% Plot quantities versus time
figure;

plot(t,v,'LineWidth',1.5)
hold on
plot(t,w,'LineWidth',1.5)
plot(t,distance,'LineWidth',1.5)
plot(t,theta,'LineWidth',1.5)
plot(t,x,'LineWidth',1.5)
plot(t,y,'LineWidth',1.5)

grid on
xlabel('Time (s)')
legend('Velocity (m/s)', ...
       'Turn rate (rad/s)', ...
       'Distance (m)', ...
       'Heading (rad)', ...
       'x (m)', ...
       'y (m)', ...
       'Location','best')

title('Basebot pose')