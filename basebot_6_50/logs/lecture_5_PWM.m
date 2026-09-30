close all;

files = {
    'device-monitor-260930-111054.log'
    'device-monitor-260930-111337.log'
};

titles = {
    'PWM = 80000 Hz'
    'PWM = 1990 Hz'
};

for k = 1:length(files)

    data = readmatrix(files{k}, 'FileType', 'text', 'CommentStyle', '%');

    t = data(:,1);

    % Voltage
    voltage_left  = data(:,4);
    voltage_right = data(:,5);

    % Velocity
    velocity_left  = data(:,8);
    velocity_right = data(:,9);

    % Current
    current_left  = data(:,10);
    current_right = data(:,11);

    figure;

    % -------- Voltage --------
    subplot(3,1,1)
    plot(t, voltage_left, 'LineWidth', 1.2)
    hold on
    plot(t, voltage_right, 'LineWidth', 1.2)
    grid on
    ylabel('Voltage [V]')
    legend('Left', 'Right')
    title(titles{k})

    % -------- Velocity --------
    subplot(3,1,2)
    plot(t, velocity_left, 'LineWidth', 1.2)
    hold on
    plot(t, velocity_right, 'LineWidth', 1.2)
    grid on
    ylabel('Velocity [rad/s]')
    legend('Left', 'Right')

    % -------- Current --------
    subplot(3,1,3)
    plot(t, current_left, 'LineWidth', 1.2)
    hold on
    plot(t, current_right, 'LineWidth', 1.2)
    grid on
    ylabel('Current [A]')
    xlabel('Time [s]')
    legend('Left', 'Right')

end