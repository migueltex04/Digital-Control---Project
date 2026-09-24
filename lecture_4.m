%% Exercise 4.4.6
% Influence of pole and zero location on the discrete step response

clear;
close all;
clc;


%% ============================================================
%  1. NOMINAL SYSTEM
%  ============================================================

zeta = 0.5;                  % Equivalent continuous-time damping ratio
theta_nom = deg2rad(45);     % Nominal pole angle [rad]

% For a discrete pole:
%
%       p = r*exp(+-j*theta)
%
% corresponding to a continuous-time damping ratio zeta:
%
%       r = exp(-zeta*theta/sqrt(1-zeta^2))

r_nom = exp(-zeta*theta_nom/sqrt(1-zeta^2));

% Nominal complex-conjugate poles
p1_nom = r_nom*exp( 1j*theta_nom);
p2_nom = r_nom*exp(-1j*theta_nom);

% Nominal zero from Eq. (4.104)
z0_nom = r_nom*cos(theta_nom);

% Nominal denominator
den_nom = [1 ...
          -2*r_nom*cos(theta_nom) ...
           r_nom^2];


fprintf('NOMINAL SYSTEM\n');
fprintf('---------------------------------\n');
fprintf('zeta          = %.2f\n',zeta);
fprintf('theta         = %.2f deg\n',rad2deg(theta_nom));
fprintf('r             = %.4f\n',r_nom);
fprintf('Nominal zero  = %.4f\n',z0_nom);
fprintf('Pole 1        = %.4f %+.4fj\n',...
    real(p1_nom),imag(p1_nom));
fprintf('Pole 2        = %.4f %+.4fj\n\n',...
    real(p2_nom),imag(p2_nom));


%% ============================================================
%  2. SIMULATION SETTINGS
%  ============================================================

Ts = 1;

% Step-response plots
N = 100;
t = 0:Ts:N*Ts;

% Longer vector for settling-time calculations
t_analysis = 0:Ts:300;

% Unit circle for z-plane plots
phi = linspace(0,2*pi,500);


%% ============================================================
%  3. ZERO STUDY
%
%  Keep poles fixed.
%  Change only z0.
%
%                 K(z-z0)
%  H(z) = ------------------------
%          z^2 - 2r cos(theta)z+r^2
%
%  K is recalculated such that:
%
%                       H(1) = 1
%
%  ============================================================


% ------------------------------------------------------------
% Three groups of three representative zeros
% ------------------------------------------------------------

z0_group1 = [-1 0 z0_nom];

z0_group2 = [0.60 0.80 0.90];

z0_group3 = [1.05 1.10 1.20];


%% ============================================================
%  ZERO GROUP 1
%  Negative / small / nominal zero
%  ============================================================

figure;
hold on;
grid on;
box on;

colors = lines(3);

for i = 1:length(z0_group1)

    z0 = z0_group1(i);

    % Unit steady-state gain
    K = (1 - 2*r_nom*cos(theta_nom) + r_nom^2)/(1-z0);

    num = K*[1 -z0];

    H = tf(num,den_nom,Ts);

    y = step(H,t);

    stairs(t,y,...
        'LineWidth',1.5,...
        'Color',colors(i,:),...
        'DisplayName',sprintf('z_0 = %.3f',z0));

end

yline(1,'k--','HandleVisibility','off');

xlabel('Sample k');
ylabel('y[k]');

title('Zero influence I - Negative to nominal zero');

xlim([0 25]);

legend('Location','best');

hold off;


%% ============================================================
%  ZERO GROUP 2
%  Positive zero approaching +1
%  ============================================================

figure;
hold on;
grid on;
box on;

colors = lines(3);

for i = 1:length(z0_group2)

    z0 = z0_group2(i);

    K = (1 - 2*r_nom*cos(theta_nom) + r_nom^2)/(1-z0);

    num = K*[1 -z0];

    H = tf(num,den_nom,Ts);

    y = step(H,t);

    stairs(t,y,...
        'LineWidth',1.5,...
        'Color',colors(i,:),...
        'DisplayName',sprintf('z_0 = %.2f',z0));

end

yline(1,'k--','HandleVisibility','off');

xlabel('Sample k');
ylabel('y[k]');

title('Zero influence II - Zero approaching z = +1');

xlim([0 20]);

legend('Location','best');

hold off;


%% ============================================================
%  ZERO GROUP 3
%  Zero outside unit circle
%  ============================================================

figure;
hold on;
grid on;
box on;

colors = lines(3);

for i = 1:length(z0_group3)

    z0 = z0_group3(i);

    K = (1 - 2*r_nom*cos(theta_nom) + r_nom^2)/(1-z0);

    num = K*[1 -z0];

    H = tf(num,den_nom,Ts);

    y = step(H,t);

    stairs(t,y,...
        'LineWidth',1.5,...
        'Color',colors(i,:),...
        'DisplayName',sprintf('z_0 = %.2f',z0));

end

yline(1,'k--','HandleVisibility','off');

xlabel('Sample k');
ylabel('y[k]');

title('Zero influence III - Zeros outside the unit circle');

xlim([0 20]);

legend('Location','best');

hold off;


%% ============================================================
%  4. Z-PLANE - ZERO STUDY
%  ============================================================

z0_all = [z0_group1 z0_group2 z0_group3];

figure;
hold on;
grid on;
box on;
axis equal;

% Unit circle
plot(cos(phi),sin(phi),'k--',...
    'LineWidth',1,...
    'DisplayName','Unit circle');


% Fixed poles
plot(real(p1_nom),imag(p1_nom),'kx',...
    'MarkerSize',12,...
    'LineWidth',2.5,...
    'DisplayName','Fixed poles');

plot(real(p2_nom),imag(p2_nom),'kx',...
    'MarkerSize',12,...
    'LineWidth',2.5,...
    'HandleVisibility','off');


% Tested zeros
plot(z0_all,zeros(size(z0_all)),'bo',...
    'MarkerSize',7,...
    'LineWidth',1.5,...
    'DisplayName','Tested zero locations');


% Nominal zero
plot(z0_nom,0,'ro',...
    'MarkerSize',10,...
    'LineWidth',2.5,...
    'DisplayName','Nominal zero');


xlabel('Real(z)');
ylabel('Imag(z)');

title('Z-plane - Zero study');

xlim([-1.3 1.3]);
ylim([-1.1 1.1]);

legend('Location','best');

hold off;



%% ============================================================
%  5. POLE STUDY
%
%  Keep zero fixed:
%
%              z0 = z0_nom
%
%  Move poles while maintaining:
%
%              zeta = 0.5
%
%  Therefore:
%
%        r = exp(-zeta*theta/sqrt(1-zeta^2))
%
%  ============================================================


theta_group1_deg = [10 20 30];

theta_group2_deg = [45 60 80];

theta_group3_deg = [100 130 160];


%% ============================================================
%  THETA GROUP 1
%  Small pole angles
%  ============================================================

figure;
hold on;
grid on;
box on;

colors = lines(3);

for i = 1:length(theta_group1_deg)

    theta = deg2rad(theta_group1_deg(i));

    % Keep zeta = 0.5
    r = exp(-zeta*theta/sqrt(1-zeta^2));

    % Denominator
    den = [1 ...
          -2*r*cos(theta) ...
           r^2];

    % Zero remains fixed
    z0 = z0_nom;

    % Unit steady-state gain
    K = (1 - 2*r*cos(theta) + r^2)/(1-z0);

    num = K*[1 -z0];

    H = tf(num,den,Ts);

    y = step(H,t);

    stairs(t,y,...
        'LineWidth',1.5,...
        'Color',colors(i,:),...
        'DisplayName',...
        sprintf('\\theta = %.0f^\\circ, r = %.3f',...
        rad2deg(theta),r));

end

yline(1,'k--','HandleVisibility','off');

xlabel('Sample k');
ylabel('y[k]');

title('Pole influence I - Small pole angles');

xlim([0 80]);

legend('Location','best');

hold off;


%% ============================================================
%  THETA GROUP 2
%  Intermediate pole angles
%  ============================================================

figure;
hold on;
grid on;
box on;

colors = lines(3);

for i = 1:length(theta_group2_deg)

    theta = deg2rad(theta_group2_deg(i));

    r = exp(-zeta*theta/sqrt(1-zeta^2));

    den = [1 ...
          -2*r*cos(theta) ...
           r^2];

    z0 = z0_nom;

    K = (1 - 2*r*cos(theta) + r^2)/(1-z0);

    num = K*[1 -z0];

    H = tf(num,den,Ts);

    y = step(H,t);

    stairs(t,y,...
        'LineWidth',1.5,...
        'Color',colors(i,:),...
        'DisplayName',...
        sprintf('\\theta = %.0f^\\circ, r = %.3f',...
        rad2deg(theta),r));

end

yline(1,'k--','HandleVisibility','off');

xlabel('Sample k');
ylabel('y[k]');

title('Pole influence II - Intermediate pole angles');

xlim([0 30]);

legend('Location','best');

hold off;


%% ============================================================
%  THETA GROUP 3
%  Large pole angles
%  ============================================================

figure;
hold on;
grid on;
box on;

colors = lines(3);

for i = 1:length(theta_group3_deg)

    theta = deg2rad(theta_group3_deg(i));

    r = exp(-zeta*theta/sqrt(1-zeta^2));

    den = [1 ...
          -2*r*cos(theta) ...
           r^2];

    z0 = z0_nom;

    K = (1 - 2*r*cos(theta) + r^2)/(1-z0);

    num = K*[1 -z0];

    H = tf(num,den,Ts);

    y = step(H,t);

    stairs(t,y,...
        'LineWidth',1.5,...
        'Color',colors(i,:),...
        'DisplayName',...
        sprintf('\\theta = %.0f^\\circ, r = %.3f',...
        rad2deg(theta),r));

end

yline(1,'k--','HandleVisibility','off');

xlabel('Sample k');
ylabel('y[k]');

title('Pole influence III - Large pole angles');

xlim([0 20]);

legend('Location','best');

hold off;



%% ============================================================
%  6. Z-PLANE - POLE STUDY
%  ============================================================

theta_all_deg = [theta_group1_deg ...
                 theta_group2_deg ...
                 theta_group3_deg];

figure;
hold on;
grid on;
box on;
axis equal;


% Unit circle
plot(cos(phi),sin(phi),'k--',...
    'LineWidth',1,...
    'DisplayName','Unit circle');


% Fixed zero
plot(z0_nom,0,'ko',...
    'MarkerSize',9,...
    'LineWidth',2.5,...
    'DisplayName',...
    sprintf('Fixed zero z_0 = %.3f',z0_nom));


% Constant damping-ratio curve
theta_curve = linspace(deg2rad(1),deg2rad(179),500);

r_curve = exp(...
    -zeta*theta_curve/sqrt(1-zeta^2));

p_curve_upper = r_curve.*exp( 1j*theta_curve);
p_curve_lower = r_curve.*exp(-1j*theta_curve);

plot(real(p_curve_upper),imag(p_curve_upper),'k:',...
    'LineWidth',1.2,...
    'DisplayName','\zeta = 0.5');

plot(real(p_curve_lower),imag(p_curve_lower),'k:',...
    'LineWidth',1.2,...
    'HandleVisibility','off');


% Tested pole locations
for i = 1:length(theta_all_deg)

    theta = deg2rad(theta_all_deg(i));

    r = exp(-zeta*theta/sqrt(1-zeta^2));

    p1 = r*exp( 1j*theta);
    p2 = r*exp(-1j*theta);

    plot(real(p1),imag(p1),'x',...
        'MarkerSize',9,...
        'LineWidth',2,...
        'DisplayName',...
        sprintf('\\theta = %.0f^\\circ',rad2deg(theta)));

    plot(real(p2),imag(p2),'x',...
        'MarkerSize',9,...
        'LineWidth',2,...
        'HandleVisibility','off');

end


xlabel('Real(z)');
ylabel('Imag(z)');

title('Z-plane - Pole locations for \zeta = 0.5');

xlim([-1.1 1.1]);
ylim([-1.1 1.1]);

legend('Location','bestoutside');

hold off;



%% ============================================================
%  7. QUANTITATIVE ANALYSIS - ZERO LOCATION
%
%  Use EXACTLY the same zero values as the response plots.
%
%  One marker in the following plots corresponds to one
%  simulated system.
%  ============================================================

z0_analysis = z0_all;

Nz = length(z0_analysis);

OS_zero = zeros(1,Nz);
Ts_zero = zeros(1,Nz);


for i = 1:Nz

    z0 = z0_analysis(i);

    K = (1 - 2*r_nom*cos(theta_nom) + r_nom^2)/(1-z0);

    num = K*[1 -z0];

    H = tf(num,den_nom,Ts);

    y = step(H,t_analysis);


    %% ---------------- Overshoot -----------------------------

    % Since y_final = 1:
    %
    % OS = (y_max - 1)/1 * 100

    OS_zero(i) = max(0,(max(y)-1)*100);

    %% ---------------- 10% settling time ----------------------

    % 2% band:
    %
    %       0.98 <= y[k] <= 1.02

    envelope = 10; % in percentage

    outside = find(abs(y-1) > envelope/100);

    if isempty(outside)

        Ts_zero(i) = 0;

    elseif outside(end) < length(t_analysis)

        Ts_zero(i) = t_analysis(outside(end)+1);

    else

        Ts_zero(i) = NaN;

    end

end

%% OVERSHOOT VS ZERO LOCATION

figure;
hold on;
grid on;
box on;

idx_left  = z0_analysis < 1;
idx_right = z0_analysis > 1;

c = [0 0.4470 0.7410];

plot(z0_analysis(idx_left),OS_zero(idx_left),'o-',...
    'Color',c,...
    'LineWidth',1.5,...
    'MarkerSize',7,...
    'HandleVisibility','off');

plot(z0_analysis(idx_right),OS_zero(idx_right),'o-',...
    'Color',c,...
    'LineWidth',1.5,...
    'MarkerSize',7,...
    'HandleVisibility','off');

xline(1,'k--','z_0 = 1');

xlabel('Zero location z_0');
ylabel('Overshoot [%]');
title('Percentage overshoot vs. zero location');

hold off;


%% SETTLING TIME VS ZERO LOCATION

figure;
hold on;
grid on;
box on;

c = [0 0.4470 0.7410];

plot(z0_analysis(idx_left),Ts_zero(idx_left),'o-',...
    'Color',c,...
    'LineWidth',1.5,...
    'MarkerSize',7,...
    'HandleVisibility','off');

plot(z0_analysis(idx_right),Ts_zero(idx_right),'o-',...
    'Color',c,...
    'LineWidth',1.5,...
    'MarkerSize',7,...
    'HandleVisibility','off');

xline(1,'k--','z_0 = 1');

xlabel('Zero location z_0');
ylabel('10% settling time [samples]');
title('10% settling time vs. zero location');

hold off;

%% ============================================================
%  8. QUANTITATIVE ANALYSIS - POLE LOCATION
%
%  Again, use exactly the theta values tested above.
%  ============================================================

theta_analysis_deg = theta_all_deg;
theta_analysis = deg2rad(theta_analysis_deg);

Nt = length(theta_analysis);

OS_theta = zeros(1,Nt);
Ts_theta = zeros(1,Nt);
r_analysis = zeros(1,Nt);


for i = 1:Nt

    theta = theta_analysis(i);

    % Maintain zeta = 0.5
    r = exp(-zeta*theta/sqrt(1-zeta^2));

    r_analysis(i) = r;

    den = [1 ...
          -2*r*cos(theta) ...
           r^2];

    % Fixed nominal zero
    z0 = z0_nom;

    % Unit steady-state gain
    K = (1 - 2*r*cos(theta) + r^2)/(1-z0);

    num = K*[1 -z0];

    H = tf(num,den,Ts);

    y = step(H,t_analysis);


    %% ---------------- Overshoot -----------------------------

    OS_theta(i) = max(0,(max(y)-1)*100);


    %% ---------------- 10% settling time ----------------------

    envelope = 10; % in percentage
    
    outside = find(abs(y-1) > envelope/100);

    if isempty(outside)

        Ts_theta(i) = 0;

    elseif outside(end) < length(t_analysis)

        Ts_theta(i) = t_analysis(outside(end)+1);

    else

        Ts_theta(i) = NaN;

    end

end


%% ============================================================
%  OVERSHOOT VS THETA
%  ============================================================

figure;

plot(theta_analysis_deg,OS_theta,'o-',...
    'LineWidth',1.5,...
    'MarkerSize',7);

grid on;
box on;

xline(rad2deg(theta_nom),'k:',...
    'Nominal \theta');

xlabel('\theta [deg]');
ylabel('Overshoot [%]');

title('Percentage overshoot vs. pole angle (\zeta = 0.5)');


%% ============================================================
%  SETTLING TIME VS THETA
%  ============================================================

figure;

plot(theta_analysis_deg,Ts_theta,'o-',...
    'LineWidth',1.5,...
    'MarkerSize',7);

grid on;
box on;

xline(rad2deg(theta_nom),'k:',...
    'Nominal \theta');

xlabel('\theta [deg]');
ylabel('10% settling time [samples]');

title('10% settling time vs. pole angle (\zeta = 0.5)');



%% ============================================================
%  9. PRINT QUANTITATIVE RESULTS
%  ============================================================

fprintf('\n');
fprintf('ZERO STUDY\n');
fprintf('-------------------------------------------------\n');
fprintf(' z0           Overshoot [%%]     Settling [samples]\n');
fprintf('-------------------------------------------------\n');

for i = 1:length(z0_analysis)

    fprintf('%7.3f        %10.3f          %6.1f\n',...
        z0_analysis(i),...
        OS_zero(i),...
        Ts_zero(i));

end


fprintf('\n');
fprintf('POLE STUDY - zeta = %.2f\n',zeta);
fprintf('-------------------------------------------------------------\n');
fprintf(' theta [deg]      r        Overshoot [%%]    Settling\n');
fprintf('-------------------------------------------------------------\n');

for i = 1:length(theta_analysis)

    fprintf('%8.0f       %.4f       %10.3f       %6.1f\n',...
        theta_analysis_deg(i),...
        r_analysis(i),...
        OS_theta(i),...
        Ts_theta(i));

end