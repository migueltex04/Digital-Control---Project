close all;

% On the floor
data = readmatrix('device-monitor-261018-215334.log', ...
    'FileType', 'text', 'CommentStyle', '%');

% Uside down
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

figure;
plot(t,distance,'LineWidth',1.2)
grid on
xlabel('Time [s]')
ylabel('Distance [m]')
title('Distance travelled')