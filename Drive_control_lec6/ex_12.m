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
J_mot  = 0.94e-6;    % kg*m^2
J = J_mot + 8.683e-6;

%% Robot geometry

gear = 9.6;
wrad = 0.0315*1.073;         % wheel radius [m]
r2m = wrad/gear;

B = 0.147;            % wheel base [m]

%% Sampling time

Ts = 0.0005;

Tsim = 14;

% Steps

t_ref = (0:0.01:Tsim)';

velocity_ref = zeros(size(t_ref));
heading_ref = zeros(size(t_ref));

v = 0.75;
T_forward = 0.75/v;
t_start = 0.5;

t_end = t_start + 5*T_forward;

velocity_ref(t_ref >= t_start & t_ref < t_end) = v;

for k = 1:4

    t_turn = t_start + k*T_forward;

    heading_ref(t_ref >= t_turn) = k*pi/2;

end

v_input = [t_ref velocity_ref];
w_input = [t_ref heading_ref];

% Slalom Trajectory

% t_ref = (0:0.01:Tsim)';
% 
% velocity_ref = 0.4 + 0.15*sin(2*pi*t_ref/10);
% turnrate_ref = 0.7*sin(2*pi*t_ref/8);
% 
% ramp = min(t_ref/1, 1);
% 
% velocity_ref = velocity_ref .* ramp;
% turnrate_ref = turnrate_ref .* ramp;
% 
% v_input = [t_ref velocity_ref];
% w_input = [t_ref turnrate_ref];

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


%% Wheel Speed Controller



% Adequate for the model
% Kp = 0.04;
% tau_I = 0.4;
% Ki = Kp/tau_I;

% Adequate for reality (it was too slow) (maybe the higher gain is useful to counter the static drag
% on the floor because with Leonora upside down it behaves differently, the Kp = 0.04 was enough)
Kp = 0.04;
tau_I = 0.16;
Ki = Kp/tau_I;


%% Turn Rate Controller

Kp_tr = 5;
tau_I_tr = 0.2;
Ki_tr = Kp_tr / tau_I_tr;

%% Heading Controller

discrete_true = -1;

Kph = 1;

num_ctrl = 1;
den_ctrl = 1;

model = 'motor_model_23b_lec8';
load_system(model);
open_system(model);

ios(1) = linio([model '/TurnRate_Controller/Uh'],1,'openinput');
ios(2) = linio([model '/Robot model'],3,'output');

setlinio(model,ios);

op = 1.0;
sys = linearize(model,ios,op);
[num, den] = ss2tf(sys.A, sys.B, sys.C, sys.D);
Gheading = minreal(tf(num, den));

disp(Gheading)
pole(Gheading)
zero(Gheading)

figure
margin(Gheading)
grid on

%% Heading lead compensator

PM_desired = 72;
PM_safety = 0;
BW_desired = 14.4/3; % Desired crossover frequency [rad/s] - One third of the closed loop frequency of the wheel speed controller

wc = BW_desired/1.5; % Initial estimate

for k = 1:30

    Gjw = squeeze(freqresp(Gheading,wc));

    mag = abs(Gjw);
    phase = rad2deg(angle(Gjw));

    if phase > 0
        phase = phase - 360;
    end

    phi_required = PM_desired + PM_safety - 180 - phase;
    phi = max(15,phi_required);

    if phi >= 80
        error('Required phase lead is too large. Reduce BW_desired.')
    end

    alpha = (1-sind(phi))/(1+sind(phi));

    Tlead = 1/(wc*sqrt(alpha));

    num_ctrl = [Tlead 1];
    den_ctrl = [alpha*Tlead 1];

    Clead = tf(num_ctrl,den_ctrl);

    Kph = 1/(mag*abs(squeeze(freqresp(Clead,wc))));

    Lheading = Kph*Clead*Gheading;
    Tclosed = feedback(Lheading,1);

    if ~isstable(Tclosed)
        error('Closed-loop unstable. Reduce BW_desired.')
    end

    BW_actual = bandwidth(Tclosed);

    if ~isfinite(BW_actual)
        error('Closed-loop bandwidth could not be determined.')
    end

    if abs(BW_actual-BW_desired)/BW_desired < 0.01
        break
    end

    wc = wc*(BW_desired/BW_actual)^0.5;

end

% Tuning

alpha = min(1,alpha*1.6);
Tlead = Tlead*0.7;
Kph = 3.4;

num_ctrl = [Tlead 1];
den_ctrl = [alpha*Tlead 1];

Clead = tf(num_ctrl,den_ctrl);

Lheading = Kph*Clead*Gheading;
Tclosed = feedback(Lheading,1);

BW_actual = bandwidth(Tclosed);

[GM,PM,Wcg,Wcp] = margin(Lheading);

fprintf('Desired bandwidth = %.2f rad/s\n',BW_desired);
fprintf('Actual bandwidth = %.2f rad/s\n',BW_actual);
fprintf('Gain crossover = %.2f rad/s\n',Wcp);
fprintf('Phase margin = %.2f deg\n',PM);
fprintf('Gain margin = %.2f dB\n',20*log10(GM));
fprintf('alpha = %.4f\n',alpha);
fprintf('Tlead = %.4f\n',Tlead);
fprintf('Kph = %.4f\n',Kph);

if abs(BW_actual-BW_desired)/BW_desired >= 0.01
    warning('Bandwidth target not reached within 30 iterations.')
end

if PM < PM_desired
    warning('The achieved phase margin is below the requirement.')
end

figure
margin(Lheading)
grid on

figure
step(Tclosed)
grid on

%% Simulink model

discrete_true = 1;

out = sim(model,Tsim);

%% Exercise 1b plot

t = out.robot.Time;
data = out.robot.Data;

% Order of signals in the Mux, as in the lecture slide:
vRef        = data(:,1);
headingRef = data(:,2);
distance    = data(:,3);
velocity    = data(:,4);
heading     = data(:,5);
turnrate    = data(:,6);

figure

plot(t,vRef,'LineWidth',1.5)
hold on
plot(t,headingRef,'LineWidth',1.5)
plot(t,distance,'LineWidth',1.5)
plot(t,velocity,'LineWidth',1.5)
plot(t,heading,'LineWidth',1.5)
plot(t,turnrate,'LineWidth',1.5)

grid on
xlabel('Time (s)')
ylabel('Value')

legend('Velocity Step (m/s)', ...   
       'Heading Step (rad/s)', ...
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