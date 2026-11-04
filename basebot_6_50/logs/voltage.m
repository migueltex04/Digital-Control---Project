%% BASEBOT - MOTOR VOLTAGE FROM LOG
% Select a 28/29-column log.
% Columns 4 and 5 contain the motor voltages.

clear;
clc;
close all;

%% 1. SELECT LOG FILE

[file, folder] = uigetfile( ...
    {'*.log;*.txt;*.csv', 'Basebot logs'}, ...
    'Select Basebot log');

if isequal(file, 0)
    disp('Selection cancelled.');
    return;
end

fid = fopen(fullfile(folder, file), 'r');

if fid < 0
    error('Could not open file: %s', file);
end

cleanupObj = onCleanup(@() fclose(fid));

%% 2. READ LOG

% Accept both 28-column and 29-column logs

data = zeros(0,5);

while ~feof(fid)

    str = fgetl(fid);

    if ~ischar(str)
        break;
    end

    values = sscanf(str, '%f')';

    if numel(values) == 28 || numel(values) == 29
        data(end+1,:) = values(1:5);
    end

end

if isempty(data)
    error('No valid samples found.');
end

%% 3. EXTRACT SIGNALS

t = data(:,1);
t = t - t(1);

uL = data(:,4);   % Left motor voltage [V]
uR = data(:,5);   % Right motor voltage [V]

%% 4. DEFINE ANALYSIS INTERVAL

% Full experiment
active = true(size(t));

% Alternatively, analyse only active movement:
% active = t >= 0.5 & t < 5.5;

%% 5. VOLTAGE LIMITS

Vlimit = 9;              % [V]
nearLimit = 0.99*Vlimit; % 8.91 V

%% 6. CALCULATE METRICS

peakL = max(abs(uL(active)));
peakR = max(abs(uR(active)));

rmsL = sqrt(mean(uL(active).^2));
rmsR = sqrt(mean(uR(active).^2));

meanL = mean(abs(uL(active)));
meanR = mean(abs(uR(active)));

satL = 100*mean(abs(uL(active)) >= nearLimit);
satR = 100*mean(abs(uR(active)) >= nearLimit);

%% 7. PRINT RESULTS

fprintf('\n========== BASEBOT MOTOR VOLTAGES ==========\n');

fprintf('Log: %s\n', file);

fprintf('Samples: %d\n', numel(t));
fprintf('Duration: %.3f s\n', t(end));

fprintf('\n');
fprintf('                         LEFT       RIGHT\n');

fprintf('Peak absolute [V]     %9.3f   %9.3f\n', ...
    peakL, peakR);

fprintf('RMS voltage [V]       %9.3f   %9.3f\n', ...
    rmsL, rmsR);

fprintf('Mean absolute [V]     %9.3f   %9.3f\n', ...
    meanL, meanR);

fprintf('Near +/-9 V [%%]      %9.2f   %9.2f\n', ...
    satL, satR);

fprintf('============================================\n');

%% 8. PLOT MOTOR VOLTAGES

figure('Color','w', ...
    'Position',[100 100 1200 580]);

hold on;

% Left motor
plot(t, uL, ...
    'b', ...
    'LineWidth',1.2, ...
    'DisplayName','Left motor');

% Right motor
plot(t, uR, ...
    'r', ...
    'LineWidth',1.2, ...
    'DisplayName','Right motor');

% Voltage limits
yline(Vlimit, ...
    'k--', ...
    '+9 V limit', ...
    'HandleVisibility','off');

yline(-Vlimit, ...
    'k--', ...
    '-9 V limit', ...
    'HandleVisibility','off');

grid on;
box on;

xlabel('Time [s]');
ylabel('Motor voltage [V]');

title('Basebot - Motor Voltages');

legend('Location','best');

xlim([t(1), max(t(end), t(1)+eps)]);

set(gca,'FontSize',11);