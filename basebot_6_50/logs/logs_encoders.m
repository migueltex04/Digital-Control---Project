%% ============================================================
% BASEBOT - HEADING CONTROL (LECTURE 8)
%
% Signals:
% 1. Velocity reference
% 2. Forward velocity
% 3. Turnrate reference
% 4. Encoder turnrate
% 5. Gyroscope turnrate (orange dashed)
% 6. Encoder heading
% 7. Heading reference
% ============================================================

clear;
clc;
close all;

%device-monitor-261030-163336

%% 1. LOAD LOG

[file, path] = uigetfile( ...
    {'*.log;*.txt;*.csv', 'Basebot logs'}, ...
    'Select Basebot log');

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

    if numel(values) == 28 || numel(values) == 29
        data(end+1, :) = values(1:28);
    end

end

fclose(fid);

if isempty(data)
    error('No valid data found.');
end

%% 2. EXTRACT SIGNALS

t = data(:,1);
t = t - t(1);

% Linear velocity
velocityRef = data(:,3);
velocity    = data(:,17);

% Turnrate
turnrateRef = data(:,16);
turnrateEnc = data(:,18);

% Gyroscope Z axis [deg/s]
gyroZ = data(:,15);

% Heading
headingEnc = data(:,21);
headingRef = data(:,28);

%% 3. GYROSCOPE TURNRATE

% Same correction used in the C++:
gyroSign = 1;
gyroTilt = 22;  % degrees

turnrateGyro = gyroSign * deg2rad(gyroZ) / cosd(gyroTilt);

%% 4. PLOT - LECTURE 8

figure('Color','w', ...
    'Position',[100 100 1250 650]);

hold on;

plot(t, velocityRef, ...
    'Color',[0.95 0.65 0.15], ...
    'LineWidth',1.3, ...
    'DisplayName','Velocity reference [m/s]');

plot(t, velocity, ...
    'Color',[0.90 0.20 0.50], ...
    'LineWidth',1.3, ...
    'DisplayName','Forward velocity [m/s]');

plot(t, turnrateRef, ...
    'Color',[0.15 0.65 0.20], ...
    'LineWidth',1.3, ...
    'DisplayName','Turnrate reference [rad/s]');

plot(t, turnrateEnc, ...
    'Color',[0.00 0.40 0.85], ...
    'LineWidth',1.3, ...
    'DisplayName','Encoder turnrate [rad/s]');

% Gyroscope turnrate - ORANGE DASHED
plot(t, turnrateGyro, ...
    'Color',[1.00 0.35 0.00], ...
    'LineStyle','--', ...
    'LineWidth',1.5, ...
    'DisplayName','Gyro turnrate [rad/s]');

plot(t, headingEnc, ...
    'Color',[0.55 0.10 0.75], ...
    'LineWidth',1.6, ...
    'DisplayName','Encoder heading [rad]');

plot(t, headingRef, ...
    'Color',[0.10 0.65 0.10], ...
    'LineStyle','--', ...
    'LineWidth',1.8, ...
    'DisplayName','Heading reference [rad]');

grid on;
box on;

xlabel('Time [s]');
ylabel('Signal value (units in legend)');

title('Basebot - Heading Control (Lecture 8)');

legend('Location','eastoutside');

xlim([t(1), t(end)]);

set(gca,'FontSize',11);

%% 5. METRICS

idxTracking = t >= 1.5 & t < 5.5;

headingError = headingRef - headingEnc;

RMSE_heading = sqrt(mean( ...
    headingError(idxTracking).^2));

lastTracking = find(idxTracking, 1, 'last');

finalHeadingError = headingError(lastTracking);

turnrateError = turnrateGyro - turnrateEnc;

RMSE_turnrate = sqrt(mean( ...
    turnrateError(idxTracking).^2));

fprintf('\n========== RESULTS ==========\n');

fprintf('Heading RMSE (encoders): %.2f deg\n', ...
    rad2deg(RMSE_heading));

fprintf('Gyro vs encoder turnrate RMSE: %.3f rad/s\n', ...
    RMSE_turnrate);

fprintf('Final heading error (encoders): %.2f deg\n', ...
    rad2deg(finalHeadingError));

fprintf('=============================\n');