% close all;

%% Load data

% On the floor
data = readmatrix('device-monitor-261029-185315', ...
    'FileType', 'text', 'CommentStyle', '%');

% Upside down
% data = readmatrix('device-monitor-261014-180713.log', ...
%     'FileType', 'text', 'CommentStyle', '%');

t = data(:,1);
vRef = data(:,3);
turnrateRef = data(:,16);
velocity = data(:,17);
turnrate = data(:,18);
x = data(:,19);
y = data(:,20);
heading = data(:,21);
distance = data(:,22);

%% Robot trajectory

figure;
plot(x,y,'LineWidth',1.5)
hold on
plot(x(1),y(1),'go','MarkerFaceColor','g')
plot(x(end),y(end),'ro','MarkerFaceColor','r')
axis equal
grid on
xlabel('x [m]')
ylabel('y [m]')
title('Robot trajectory')
legend('Trajectory','Start','End','Location','best')
xlim([0 1.0]);

%% Velocity, turnrate and heading

figure;

subplot(3,1,1)
plot(t,vRef,'--',t,velocity,'LineWidth',1.2)
grid on
ylabel('Velocity [m/s]')
legend('Reference','Measured','Location','best')

subplot(3,1,2)
plot(t,turnrateRef,'--',t,turnrate,'LineWidth',1.2)
grid on
ylabel('Turn rate [rad/s]')
legend('Reference','Measured','Location','best')

subplot(3,1,3)
plot(t,heading,'LineWidth',1.2)
grid on
ylabel('Heading [rad]')
xlabel('Time [s]')

%% Distance travelled

figure;
plot(t,distance,'LineWidth',1.2)
grid on
xlabel('Time [s]')
ylabel('Distance [m]')
title('Distance travelled')

%% Velocity performance metrics

% Steady-state interval
idx = (t >= 0.5) & (t <= 3.0);

v_ss = velocity(idx);
v_ref_ss = vRef(idx);

% 1. Maximum and minimum velocity
v_max = max(v_ss);
v_min = min(v_ss);

% 2. Peak-to-peak amplitude
delta_v = abs(v_max - v_min);

% 3. Standard deviation
sigma_v = std(v_ss);

% 4. RMSE relative to reference
rmse_v = sqrt(mean((v_ss - v_ref_ss).^2));

%% Settling time (5% tolerance)

% Detect positive reference step
step_idx = find(diff(vRef) > 0.1, 1, 'first') + 1;

settling_time = NaN;

if ~isempty(step_idx)

    t_step = t(step_idx);
    v_target = vRef(step_idx);

    % Find when reference returns to zero
    stop_idx = find(vRef(step_idx:end) < 0.5*v_target, ...
                    1, 'first');

    if isempty(stop_idx)
        end_idx = length(t);
    else
        end_idx = step_idx + stop_idx - 2;
    end

    % Smooth measured velocity (100 ms moving average)
    dt = median(diff(t));
    window = max(1, round(0.10/dt));

    % Smooth only the constant-reference interval
    % to avoid contamination from the stopping transient
    v_smooth = velocity;
    v_smooth(step_idx:end_idx) = ...
        movmean(velocity(step_idx:end_idx), window);

    % Settling band: +/- 5%
    tolerance = 0.05;

    lower = v_target*(1-tolerance);
    upper = v_target*(1+tolerance);

    % Ignore first 50 ms after the step
    start_idx = find(t >= t_step + 0.05, 1, 'first');

    if ~isempty(start_idx) && start_idx <= end_idx

        % Find all samples outside the settling band
        idx_settle = start_idx:end_idx;

        outside = find( ...
            v_smooth(idx_settle) < lower | ...
            v_smooth(idx_settle) > upper);

        if isempty(outside)

            settling_time = t(start_idx) - t_step;

        elseif outside(end) < numel(idx_settle)

            settling_idx = idx_settle(outside(end) + 1);
            settling_time = t(settling_idx) - t_step;

        end
    end

    %% Settling time plot

    figure;

    plot(t, velocity, ...
        'Color', [0.8 0.8 0.8], ...
        'LineWidth', 0.8);
    hold on;

    plot(t, v_smooth, 'b', 'LineWidth', 1.5);

    yline(v_target, 'k--', 'Reference');
    yline(lower, 'r--', '-5%');
    yline(upper, 'r--', '+5%');

    if ~isnan(settling_time)
        xline(t_step + settling_time, ...
            'g--', 'Settling time', ...
            'LineWidth', 1.5);
    end

    xlim([0 min(1.5,t(end))]);
    grid on
    xlabel('Time [s]')
    ylabel('Velocity [m/s]')
    title('Velocity settling time (5% tolerance)')
    legend('Measured', 'Smoothed', ...
           'Reference', '-5%', '+5%', ...
           'Location', 'best')

end

%% Print results

fprintf('\n===== VELOCITY PERFORMANCE =====\n');

fprintf('Maximum velocity:       %.5f m/s\n', v_max);
fprintf('Minimum velocity:       %.5f m/s\n', v_min);
fprintf('Peak-to-peak amplitude: %.5f m/s\n', delta_v);
fprintf('Standard deviation:     %.5f m/s\n', sigma_v);
fprintf('RMSE:                   %.5f m/s\n', rmse_v);

if ~isnan(settling_time)
    fprintf('Settling time (5%%):     %.4f s\n', settling_time);
else
    fprintf('Settling time (5%%):     Not reached\n');
end