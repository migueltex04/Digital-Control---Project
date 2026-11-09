
%% Exercise 1b - Velocity and heading control

%% Motor parameters

L  = 3.57e-3;        % H
R  = 3.57;           % Ohm
Kt = 0.008863;       % Nm/A
Kb = Kt;
D  = 8.57e-7;        % Nm/(rad/s)
J_mot = 0.94e-6;     % kg*m^2
J = J_mot + 8.683e-6;

%% Robot geometry

gear = 9.6;
wrad = 0.0315*1.073;
r2m = wrad/gear;

B = 0.147;

%% Sampling time

Ts = 0.008;

%% Reference - Square mission

Tsim = 12;

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

%v_input = [t_ref velocity_ref];
%w_input = [t_ref heading_ref];

%% Wheel Speed Controller

Kp = 0.04;
tau_I = 0.16;
Ki = Kp/tau_I;

%% Turn Rate Controller

Kp_tr = 1;
tau_I_tr = 0.2;
Ki_tr = Kp_tr/tau_I_tr;

%% Acceleration Controller

g = 9.81;
miu = 0.6; % match with the leonora_fast_mission but test with smaller values (like 0.4 or even 0.3) and then test on the floor
aup = miu*g;
adown = -miu*g;
Kp_acc  = 25;

%% Model Linearization Analysis

discrete_true = -1;

Kph = 1;

num_ctrl = 1;
den_ctrl = 1;

model = 'motor_model_23b_lec9_TR';

load_system(model);
open_system(model);

%% Turn Rate Controller - Closed-loop bandwidth

% The output of Heading_Controller is omega_ref.
% Open the connection to the outer heading loop at that signal.
ios_tr(1) = linio([model '/Heading_Controller'], ...
                 1, 'openinput');

% Robot model output 4 = actual turn rate.
% Keep the turn-rate feedback loop CLOSED.
ios_tr(2) = linio([model '/Robot model'], ...
                 4, 'output');

opt_tr = linearizeOptions;
opt_tr.SampleTime = 0;

% Initial rest condition, before the mission starts.
op_tr = 0;

sys_tr = linearize(model, ios_tr, op_tr, opt_tr);

if sys_tr.Ts ~= 0
    error('Turn-rate linearization is not continuous-time.')
end

Ttr = minreal(tf(sys_tr));

if ~isstable(Ttr)
    error('The turn-rate closed loop is unstable.')
end

BW_tr = bandwidth(Ttr);
info_tr = stepinfo(Ttr);

fprintf('\n--- Turn Rate Closed Loop ---\n');
disp(Ttr)

fprintf('DC gain = %.3f\n', dcgain(Ttr));
fprintf('Bandwidth = %.3f rad/s\n', BW_tr);
fprintf('Rise time = %.3f s\n', info_tr.RiseTime);
fprintf('Settling time = %.3f s\n', info_tr.SettlingTime);
fprintf('Overshoot = %.2f %%\n', info_tr.Overshoot);

figure
bode(Ttr)
grid on
title('Turn Rate - Closed-loop Frequency Response')

figure
step(Ttr)
grid on
title('Turn Rate - Closed-loop Step Response')

%% Heading Controller - Linearization

ios(1) = linio([model '/Heading_Controller/Uh'],1,'openinput');
ios(2) = linio([model '/Robot model'],3,'output');

setlinio(model,ios);

op = 1.0;

opt = linearizeOptions;
opt.SampleTime = 0;

sys = linearize(model,ios,op,opt);

if sys.Ts ~= 0
    error('The linearized system is not continuous-time.')
end

Gheading = minreal(tf(sys));

disp(Gheading)
pole(Gheading)
zero(Gheading)

figure
margin(Gheading)
grid on
title('Heading Plant - Bode Diagram')

%% Heading Phase-Lead Controller Design

PM_desired = 72;
PM_safety = 0;
BW_desired = 14.4/3; % Desired closed-loop bandwidth [rad/s]

wc = BW_desired/1.5;

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

%% Manual Controller Tuning

Kph = 4.1; % tune not to have so much overshoot
alpha = min(1,alpha*1.6); 
Tlead = Tlead*1; 

num_ctrl = [Tlead 1];
den_ctrl = [alpha*Tlead 1];

Clead = tf(num_ctrl,den_ctrl);

Lheading = Kph*Clead*Gheading;
Tclosed = feedback(Lheading,1);

%% Heading Controller Performance

BW_actual = bandwidth(Tclosed);

[GM,PM,Wcg,Wcp] = margin(Lheading);

fprintf('\n--- Heading Controller ---\n');

fprintf('Desired bandwidth = %.2f rad/s\n',BW_desired);
fprintf('Actual bandwidth = %.2f rad/s\n',BW_actual);
fprintf('Gain crossover = %.2f rad/s\n',Wcp);
fprintf('Phase margin = %.2f deg\n',PM);
fprintf('Gain margin = %.2f dB\n',20*log10(GM));

fprintf('\n--- Controller Parameters ---\n');

fprintf('Kph = %.6f\n',Kph);
fprintf('Tlead = %.6f\n',Tlead);
fprintf('alpha = %.6f\n',alpha);

fprintf('\nNumerator: ');
disp(num_ctrl)

fprintf('Denominator: ');
disp(den_ctrl)

if abs(BW_actual-BW_desired)/BW_desired >= 0.01
    warning('Bandwidth target not reached within 30 iterations.')
end

if PM < PM_desired
    warning('The achieved phase margin is below the requirement.')
end

if ~isstable(Tclosed)
    warning('The closed-loop system is unstable.')
end

figure
margin(Lheading)
grid on
title('Compensated Open-Loop System')

figure
step(Tclosed)
grid on
title('Heading Closed-Loop Step Response')

info = stepinfo(Tclosed);

fprintf('\n--- Step Response ---\n');
fprintf('Rise time = %.3f s\n',info.RiseTime);
fprintf('Settling time = %.3f s\n',info.SettlingTime);
fprintf('Overshoot = %.2f %%\n',info.Overshoot);

%% Simulink Model

discrete_true = 1;

out = sim(model,Tsim);

%% Exercise 1b Plot

t = out.robot.Time;
data = out.robot.Data;

vRef       = data(:,1);
headingRef = data(:,2);
distance   = data(:,3);
velocity   = data(:,4);
heading    = data(:,5);
turnrate   = data(:,6);

figure

plot(t,vRef,'LineWidth',1.5)
hold on
plot(t,headingRef,'LineWidth',1.5)
plot(t,distance,'LineWidth',1.5)
plot(t,velocity,'LineWidth',1.5)
plot(t,heading,'LineWidth',1.5)
plot(t,turnrate,'LineWidth',1.5)
plot(turnrate_input(:,1), turnrate_input(:,2), 'LineWidth',1.5)

grid on
xlabel('Time (s)')
ylabel('Value')

legend('Velocity reference (m/s)', ...
       'Heading reference (rad)', ...
       'Robot distance (m)', ...
       'Robot velocity (m/s)', ...
       'Robot heading (rad)', ...
       'Robot turnrate (rad/s)', ...
       'Turnrate reference (rad/s)', ...
       'Location','best')

title('Exercise 1b - Velocity and Heading Control')

%% Heading Tracking

figure

plot(t,headingRef,'--','LineWidth',1.5)
hold on
plot(t,heading,'LineWidth',1.5)

grid on
xlabel('Time (s)')
ylabel('Heading (rad)')
legend('Reference','Actual','Location','best')
title('Heading Tracking')

%% Motor Inputs

t_ML = out.velP.Time;
data_ML = out.velP.Data;

t_MR = out.velPR.Time;
data_MR = out.velPR.Data;

input_ML = data_ML(:,5);
input_MR = data_MR(:,5);

figure

plot(t_ML,input_ML,'LineWidth',1.5)
hold on
plot(t_MR,input_MR,'LineWidth',1.5)

grid on
xlabel('Time (s)')
ylabel('Motor input (V)')
legend('Left motor','Right motor','Location','best')
title('Actuator Inputs')

%% Robot Trajectory

x = cumtrapz(t,velocity.*cos(heading));
y = cumtrapz(t,velocity.*sin(heading));

figure

plot(x,y,'b','LineWidth',1.5)
hold on
plot(0,0,'go','MarkerFaceColor','g')
plot(x(end),y(end),'ro','MarkerFaceColor','r')

grid on
axis equal
xlabel('X (m)')
ylabel('Y (m)')

legend('Robot trajectory','Initial position','Final position', ...
       'Location','best')

title('Robot Trajectory in the XY Plane')

%% Acceleration Plots

t_acc  = out.accTurnVel.Time;
accVel_data = out.accTurnVel.Data;

accVel = accVel_data(:,2);

figure

plot(t_acc,accVel,'LineWidth',1.5)

grid on
xlabel('Time (s)')
ylabel('Linear Acceleration (m/s^2)')
legend('Acceleration','Location','best')
title('Acceleration')

t_acc_lat  = out.accTurnHead.Time;
accLat_data = out.accTurnHead.Data;

accLat = accLat_data(:,2);

figure

plot(t_acc_lat,accLat,'LineWidth',1.5)

grid on
xlabel('Time (s)')
ylabel('Rotating Acceleration (m/s^2)')
legend('Acceleration','Location','best')
title('Acceleration')