%% ============================================================
% BASEBOT - HEADING CONTROL WITH GYROSCOPE FEEDBACK
% DTU DIGITAL CONTROL - LECTURE 8
%
% C++ LOG FORMAT: 29 COLUMNS
%
% Heading feedback: GYROSCOPE
% Wheel velocity feedback: ENCODERS
%
% FIGURE 1:
% All signals superimposed (Lecture 8)
%
% FIGURE 2:
% XY trajectories:
%   - Reference
%   - Encoder odometry
%   - Gyroscope heading + encoder forward velocity
%
% IMPORTANT:
% Gyroscope heading is already integrated in C++.
% Do NOT integrate gyroZ again for the feedback heading.
% ============================================================


%device-monitor-261102-182150
clear;
clc;
close all;

%% 1. LOAD LOG

[file, path] = uigetfile( ...
    {'*.log;*.txt;*.csv', 'Basebot logs'}, ...
    'Select Basebot gyro-feedback log');

if isequal(file, 0)
    error('No file selected.');
end

fid = fopen(fullfile(path, file), 'r');

if fid < 0
    error('Could not open log file.');
end

data = [];

while ~feof(fid)

    line = fgetl(fid);

    if ~ischar(line)
        break;
    end

    line = strtrim(line);

    if isempty(line) || startsWith(line, '%')
        continue;
    end

    values = sscanf(line, '%f')';

    % This experiment must have 29 columns
    if numel(values) == 29
        data(end+1, :) = values;
    end

end

fclose(fid);

if isempty(data)
    error(['No valid data found. ' ...
           'This script requires the 29-column gyro-feedback log.']);
end

%% 2. EXTRACT DATA

t = data(:,1);
t = t - t(1);

state = data(:,2);

% Linear velocity
vRef = data(:,3);
v    = data(:,17);

% Angular velocity
turnrateRef = data(:,16);
turnrateEnc = data(:,18);

% Raw gyro Z [deg/s]
gyroZ = data(:,15);

% Headings [rad]
headingEnc  = data(:,21);
headingRef  = data(:,28);
headingGyro = data(:,29);

% Encoder position [m]
xEnc = data(:,19);
yEnc = data(:,20);

%% 3. GYROSCOPE TURNRATE CORRECTION

% The C++ uses:
% gyroSign = +1
% tiltAngle = 22 degrees
%
% imu2.gyro[2] is already calibrated by uimu2.cpp.
%
% Do not estimate a new bias using the first 0.4 seconds,
% because the signal can contain vibrations/movement.

gyroSign = 1;
tiltAngle = 22;

turnrateGyro = ...
    gyroSign * deg2rad(gyroZ) / cosd(tiltAngle);

%% 4. DEFINE VALID TRACKING INTERVAL

% state == 2 corresponds to the stopping phase.
%
% At the beginning of that phase, C++ changes:
%
% desiredHeading = headingGyro;
%
% Therefore, the final 0.5 seconds must not be included
% when calculating the tracking performance.

idxTracking = state > 0 & state ~= 2;

if nnz(idxTracking) < 10
    error('Not enough tracking samples.');
end

% Last sample before stopping
lastTracking = find(idxTracking, 1, 'last');

%% 5. HEADING TRACKING METRICS

% IMPORTANT:
% This is the actual heading error used by the controller.

headingErrorGyro = headingRef - headingGyro;

% Encoder error only for comparison
headingErrorEnc = headingRef - headingEnc;

% Gyro tracking RMSE
RMSE_headingGyro = sqrt(mean( ...
    headingErrorGyro(idxTracking).^2));

% Encoder heading RMSE
RMSE_headingEnc = sqrt(mean( ...
    headingErrorEnc(idxTracking).^2));

% Maximum gyro heading error
maxHeadingErrorGyro = max( ...
    abs(headingErrorGyro(idxTracking)));

% Final tracking errors before stopping
finalHeadingErrorGyro = ...
    headingErrorGyro(lastTracking);

finalHeadingErrorEnc = ...
    headingErrorEnc(lastTracking);

%% 6. GYRO VS ENCODER TURNRATE

turnrateError = turnrateGyro - turnrateEnc;

RMSE_turnrate = sqrt(mean( ...
    turnrateError(idxTracking).^2));

%% 7. PRINT RESULTS

fprintf('\n========== BASEBOT - GYRO FEEDBACK ==========\n');

fprintf('Samples: %d\n', size(data,1));

fprintf('Tracking duration: %.3f s\n', ...
    t(lastTracking));

fprintf('\nHEADING TRACKING\n');

fprintf('Gyro heading RMSE: %.2f deg\n', ...
    rad2deg(RMSE_headingGyro));

fprintf('Encoder heading RMSE: %.2f deg\n', ...
    rad2deg(RMSE_headingEnc));

fprintf('Maximum gyro heading error: %.2f deg\n', ...
    rad2deg(maxHeadingErrorGyro));

fprintf('\nFINAL HEADING BEFORE STOP\n');

fprintf('Reference: %.2f deg\n', ...
    rad2deg(headingRef(lastTracking)));

fprintf('Gyroscope: %.2f deg\n', ...
    rad2deg(headingGyro(lastTracking)));

fprintf('Encoders: %.2f deg\n', ...
    rad2deg(headingEnc(lastTracking)));

fprintf('Gyro final error: %.2f deg\n', ...
    rad2deg(finalHeadingErrorGyro));

fprintf('Encoder final error: %.2f deg\n', ...
    rad2deg(finalHeadingErrorEnc));

fprintf('\nTURNRATE COMPARISON\n');

fprintf('Gyro vs encoder RMSE: %.3f rad/s\n', ...
    RMSE_turnrate);

fprintf('============================================\n');

%% 8. FIGURE 1 - LECTURE 8 (ALL SIGNALS)

figure('Color','w', ...
    'Position',[100 100 1250 650]);

hold on;

plot(t, vRef, ...
    'Color',[0.95 0.65 0.15], ...
    'LineWidth',1.2, ...
    'DisplayName','Velocity reference [m/s]');

plot(t, v, ...
    'Color',[0.90 0.20 0.50], ...
    'LineWidth',1.2, ...
    'DisplayName','Forward velocity [m/s]');

plot(t, turnrateRef, ...
    'Color',[0.15 0.65 0.20], ...
    'LineWidth',1.3, ...
    'DisplayName','Turnrate command [rad/s]');

plot(t, turnrateEnc, ...
    'Color',[0.00 0.40 0.85], ...
    'LineWidth',1.1, ...
    'DisplayName','Encoder turnrate [rad/s]');

plot(t, turnrateGyro, ...
    'Color',[1.00 0.35 0.00], ...
    'LineStyle','--', ...
    'LineWidth',1.5, ...
    'DisplayName','Gyro turnrate [rad/s]');


plot(t, headingGyro, ...
    'Color',[0.00 0.65 0.65], ...
    'LineWidth',1.8, ...
    'DisplayName','Gyroscope heading [rad]');

plot(t, headingRef, ...
    'Color',[0.10 0.65 0.10], ...
    'LineStyle','--', ...
    'LineWidth',1.8, ...
    'DisplayName','Heading reference [rad]');

grid on;
box on;

xlabel('Time [s]');
ylabel('Signal value (units in legend)');

title('Basebot - Heading Control with Gyroscope Feedback');

legend('Location','eastoutside');

xlim([t(1) t(end)]);

set(gca,'FontSize',11);

%% 9. RECONSTRUCT XY TRAJECTORIES

% We compare three trajectories:
%
% 1. Ideal reference:
%    vRef and headingRef
%
% 2. Encoder odometry:
%    xEnc and yEnc from C++
%
% 3. Gyroscope-based trajectory:
%    encoder forward velocity + gyro heading
%
% The gyroscope measures orientation, not position.
% Therefore, forward velocity is still taken from encoders.

% Reference trajectory

xRef = xEnc(1) + cumtrapz( ...
    t, vRef .* cos(headingRef));

yRef = yEnc(1) + cumtrapz( ...
    t, vRef .* sin(headingRef));

% Gyroscope-based position reconstruction

xGyro = xEnc(1) + cumtrapz( ...
    t, v .* cos(headingGyro));

yGyro = yEnc(1) + cumtrapz( ...
    t, v .* sin(headingGyro));

%% 10. TRAJECTORY METRICS

% Compare positions at corresponding times
% during the active trajectory.

positionErrorGyro = hypot( ...
    xGyro-xRef, yGyro-yRef);

positionErrorEnc = hypot( ...
    xEnc-xRef, yEnc-yRef);

RMSE_positionGyro = sqrt(mean( ...
    positionErrorGyro(idxTracking).^2));

RMSE_positionEnc = sqrt(mean( ...
    positionErrorEnc(idxTracking).^2));

finalPositionErrorGyro = ...
    positionErrorGyro(lastTracking);

finalPositionErrorEnc = ...
    positionErrorEnc(lastTracking);

fprintf('\n========== TRAJECTORY COMPARISON ==========\n');

fprintf('Gyro-based position RMSE: %.4f m\n', ...
    RMSE_positionGyro);

fprintf('Encoder position RMSE: %.4f m\n', ...
    RMSE_positionEnc);

fprintf('Gyro-based final position error: %.4f m\n', ...
    finalPositionErrorGyro);

fprintf('Encoder final position error: %.4f m\n', ...
    finalPositionErrorEnc);

fprintf('===========================================\n');

%% 11. FIGURE 2 - XY TRAJECTORIES

figure('Color','w', ...
    'Position',[150 150 900 700]);

hold on;

% Restrict trajectory comparison to active tracking

idxPlot = 1:lastTracking;

plot(xRef(idxPlot), yRef(idxPlot), ...
    'k--', ...
    'LineWidth',2, ...
    'DisplayName','Ideal reference');

plot(xEnc(idxPlot), yEnc(idxPlot), ...
    'Color',[0.10 0.40 0.90], ...
    'LineWidth',1.8, ...
    'DisplayName','Encoder odometry');

plot(xGyro(idxPlot), yGyro(idxPlot), ...
    'Color',[0.90 0.20 0.20], ...
    'LineWidth',2, ...
    'DisplayName','Gyroscope heading + encoder velocity');

% Starting point

plot(xRef(1), yRef(1), ...
    'go', ...
    'MarkerFaceColor','g', ...
    'MarkerSize',8, ...
    'DisplayName','Start');

% Endpoints

plot(xRef(lastTracking), yRef(lastTracking), ...
    'ks', ...
    'MarkerFaceColor','k', ...
    'MarkerSize',7, ...
    'DisplayName','Reference end');

plot(xGyro(lastTracking), yGyro(lastTracking), ...
    'ro', ...
    'MarkerFaceColor','r', ...
    'MarkerSize',7, ...
    'DisplayName','Gyro-based end');

axis equal;
grid on;
box on;

xlabel('X [m]');
ylabel('Y [m]');

title('Basebot - Trajectory Comparison');

legend('Location','best');

set(gca,'FontSize',11);

%% HEADING CONTROLLER - SETTLING TIME ANALYSIS

% Signals
t = data(:,1);
t = t - t(1);

headingRef  = data(:,28);
headingGyro = data(:,29);

% Four 90-degree heading steps
stepTimes = [1.5 2.5 3.5 4.5];

stepDuration = 1.0;

stepAmplitude = pi/2;

% Settling tolerance: 5% of 90 degrees
tolerance = 0.05 * stepAmplitude;

settlingTimes = NaN(1,4);
overshootDeg = NaN(1,4);
finalErrorDeg = NaN(1,4);

for k = 1:4

    tStart = stepTimes(k);
    tEnd   = tStart + stepDuration;

    % Use samples before the next reference change
    idx = find(t >= tStart & t < tEnd);

    if isempty(idx)
        continue;
    end

    tk = t(idx);
    theta = headingGyro(idx);

    reference = k * pi/2;

    error = reference - theta;

    % Settling time:
    % First instant after which ALL remaining samples
    % stay inside the tolerance band.

    inside = abs(error) <= tolerance;

    for j = 1:length(idx)

        if all(inside(j:end))

            settlingTimes(k) = tk(j) - tStart;

            break;
        end
    end

    % Overshoot relative to the new reference
    overshootDeg(k) = rad2deg( ...
        max(0, max(theta-reference)));

    % Error before the next heading step
    finalErrorDeg(k) = rad2deg(error(end));

end

%% OVERALL RMSE

idxTracking = t >= 1.5 & t < 5.5;

headingError = headingRef - headingGyro;

RMSE_heading = rad2deg( ...
    sqrt(mean(headingError(idxTracking).^2)));

%% PRINT RESULTS

fprintf('\n========== HEADING CONTROLLER ==========\n');

fprintf('Heading RMSE: %.3f deg\n',RMSE_heading);

fprintf('\nSETTLING TIMES (5%% = +/-4.5 deg)\n');

for k = 1:4

    fprintf('Step %d: %.3f s\n', ...
        k,settlingTimes(k));

end

fprintf('\nOVERSHOOT\n');

for k = 1:4

    fprintf('Step %d: %.2f deg\n', ...
        k,overshootDeg(k));

end

fprintf('\nFINAL ERRORS\n');

for k = 1:4

    fprintf('Step %d: %.2f deg\n', ...
        k,finalErrorDeg(k));

end

fprintf('\nMean settling time: %.3f s\n', ...
    mean(settlingTimes,'omitnan'));

fprintf('Max overshoot: %.2f deg\n', ...
    max(overshootDeg));

fprintf('========================================\n');

%% 12. FIGURE 3 - X(t) AND Y(t)

figure('Color','w', ...
    'Position',[200 150 1100 750]);

% Use only the active trajectory
idxPlot = 1:lastTracking;

%% X POSITION

subplot(2,1,1);
hold on;

plot(t(idxPlot), xRef(idxPlot), ...
    'k--', ...
    'LineWidth',2, ...
    'DisplayName','Reference');

plot(t(idxPlot), xEnc(idxPlot), ...
    'Color',[0.10 0.40 0.90], ...
    'LineWidth',1.5, ...
    'DisplayName','Encoder odometry');

plot(t(idxPlot), xGyro(idxPlot), ...
    'Color',[0.90 0.20 0.20], ...
    'LineWidth',1.8, ...
    'DisplayName','Gyroscope-based estimate');

grid on;
box on;

xlabel('Time [s]');
ylabel('X [m]');

title('Basebot - X Position');

legend('Location','best');

xlim([t(1) t(lastTracking)]);

%% Y POSITION

subplot(2,1,2);
hold on;

plot(t(idxPlot), yRef(idxPlot), ...
    'k--', ...
    'LineWidth',2, ...
    'DisplayName','Reference');

plot(t(idxPlot), yEnc(idxPlot), ...
    'Color',[0.10 0.40 0.90], ...
    'LineWidth',1.5, ...
    'DisplayName','Encoder odometry');

plot(t(idxPlot), yGyro(idxPlot), ...
    'Color',[0.90 0.20 0.20], ...
    'LineWidth',1.8, ...
    'DisplayName','Gyroscope-based estimate');

grid on;
box on;

xlabel('Time [s]');
ylabel('Y [m]');

title('Basebot - Y Position');

legend('Location','best');

xlim([t(1) t(lastTracking)]);

sgtitle('Basebot - Position Tracking with Gyroscope Feedback');