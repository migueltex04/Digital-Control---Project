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

    voltage_desired = data(:,3);

    velocity_left  = data(:,8);
    velocity_right = data(:,9);

    current_left  = data(:,10);
    current_right = data(:,11);

    figure

    plot(t, voltage_desired/10, 'LineWidth', 1.2)
    hold on
    plot(t, velocity_left/1000, 'LineWidth', 1.2)
    plot(t, velocity_right/1000, 'LineWidth', 1.2)
    plot(t, current_left, 'LineWidth', 1.2)
    plot(t, current_right, 'LineWidth', 1.2)

    grid on
    xlabel('Time [s]')
    ylabel('Scaled quantities')
    title(titles{k})

    legend( ...
        'Desired voltage / 10', ...
        'Left velocity [krad/s]', ...
        'Right velocity [krad/s]', ...
        'Left current [A]', ...
        'Right current [A]' ...
    )

end