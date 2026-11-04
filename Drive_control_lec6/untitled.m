%% BASEBOT - TURNRATE PI TUNING AROUND ORIGINAL GAINS
% Original Basebot vs Fast 3:1 vs nearby Kp/Ki combinations
%
% Discrete nonlinear simulation:
%   Ts = 0.0005 s
%   Inner wheel PI: Kp = 0.08, Ki = 0.2
%   Outer turnrate PI: variable Kp, Ki
%   Turnrate command saturation: +/- 4 rad/s
%   Motor voltage saturation: +/- 9 V
%   Trapezoidal PI integration with conditional anti-windup
%
% No imposed 3:1 or 4:1 bandwidth ratio.
%
% IMPORTANT:
% This uses a simplified model with two independent motors.
% The equivalent inertia is the approximation used previously.
% Robot yaw inertia and encoder noise are not explicitly modeled.
% Results must be validated on the real robot.
%
% Requires Control System Toolbox.

clear;
close all;
clc;

%% 1. MOTOR PARAMETERS

L  = 3.57e-3;        % H
R  = 3.57;           % Ohm
Kt = 0.008863;       % Nm/A
Kb = Kt;
D  = 8.57e-7;        % Nm/(rad/s)

Jmotor = 0.94e-6;    % kg*m^2

%% 2. ROBOT PARAMETERS

M = 0.904;           % kg

gear = 9.6;
wheelRadius = 0.03380;  % m
wheelBase = 0.147;      % m

Ts = 0.0005;         % s

Vmax = 9.0;          % V
turnrateMax = 4.0;   % rad/s

% Approximate equivalent inertia referred to motor shaft

Jrobot = M * (wheelRadius/gear)^2;
J = Jmotor + Jrobot;

fprintf('============================================\n');
fprintf(' ROBOT PARAMETERS\n');
fprintf('============================================\n');

fprintf('Motor inertia: %.8e kg*m^2\n', Jmotor);
fprintf('Robot equivalent inertia: %.8e kg*m^2\n', Jrobot);
fprintf('Total inertia: %.8e kg*m^2\n\n', J);

%% 3. INNER WHEEL PI

KpWheel = 0.08;
tauiWheel = 0.4;
KiWheel = KpWheel/tauiWheel;

%% 4. REFERENCE OUTER CONTROLLERS

% Original Basebot
KpOriginal = 5.0;
KiOriginal = 25.0;

% Fast 3:1
KpFast = 0.0443322593;
tauiFast = 0.00778527218;
KiFast = KpFast/tauiFast;

%% 5. CONTINUOUS MOTOR MODEL

s = tf('s');

% Motor voltage -> motor angular velocity

Gmotor = Kt / ( ...
    J*L*s^2 + ...
    (D*L + J*R)*s + ...
    (D*R + Kt*Kb));

% Inner wheel controller

Cwheel = KpWheel + KiWheel/s;

% Closed-loop wheel dynamics

Twheel = minreal(feedback(Cwheel*Gmotor,1));

if ~isstable(Twheel)
    error('The inner wheel loop is unstable.');
end

BWinner = bandwidth(Twheel);

% Simplified outer turnrate plant:
% Assumes two identical, independent inner wheel loops.

Gturn = Twheel;

fprintf('Inner wheel bandwidth: %.4f rad/s\n\n', BWinner);

%% 6. DISCRETIZE MOTOR MODEL ONCE

% State vector for each motor:
% x = [current; angular velocity]
%
% di/dt     = (-R*i - Kb*omega + u)/L
% domega/dt = (Kt*i - D*omega)/J

A = [ ...
    -R/L,   -Kb/L; ...
     Kt/J,  -D/J];

B = [1/L; 0];

motorSS = ss(A,B,eye(2),zeros(2,1));

motorDiscrete = c2d(motorSS,Ts,'zoh');

Ad = motorDiscrete.A;
Bd = motorDiscrete.B;

%% 7. SIMULATION PARAMETERS

p.L = L;
p.R = R;
p.Kt = Kt;
p.Kb = Kb;
p.D = D;
p.J = J;

p.gear = gear;
p.wheelRadius = wheelRadius;
p.wheelBase = wheelBase;

p.Ts = Ts;
p.Vmax = Vmax;
p.turnrateMax = turnrateMax;

p.KpWheel = KpWheel;
p.KiWheel = KiWheel;

% Precomputed discrete motor matrices
p.Ad = Ad;
p.Bd = Bd;

%% 8. SEARCH GRID

% Neighborhood around original Basebot gains.
%
% Start with 21 points in each direction.
% Add original gains explicitly if missing.

KpValues = linspace(1,10,21);
KiValues = linspace(5,50,21);

KpValues = unique([KpValues KpOriginal]);
KiValues = unique([KiValues KiOriginal]);

nKp = numel(KpValues);
nKi = numel(KiValues);

nCases = nKp*nKi;

%% 9. TEST DEFINITION

% Same reference sequence as firmware:
%
% 0.0 - 0.5 s: vref = 0,   wref = 0
% 0.5 - 2.0 s: vref = 0.3, wref = 0
% 2.0 - 4.0 s: vref = 0.3, wref = 0.5
% 4.0 - 4.5 s: vref = 0,   wref = 0

Tend = 4.5;

t = (0:Ts:Tend)';

vReference = zeros(size(t));
wReference = zeros(size(t));

vReference(t>=0.5 & t<4.0) = 0.3;
wReference(t>=2.0 & t<4.0) = 0.5;

stepStart = 2.0;
stepEnd = 4.0;

stepMask = t>=stepStart & t<stepEnd;

% First 0.5 seconds after step

transientMask = ...
    t>=stepStart & t<=stepStart+0.5;

% Heading reference, calculated once

thetaReference = cumtrapz(t,wReference);

idxStepEnd = find(t<stepEnd,1,'last');

%% 10. ALLOCATE RESULTS

KpResult = nan(nCases,1);
KiResult = nan(nCases,1);

BWResult = nan(nCases,1);
NResult = nan(nCases,1);
PMResult = nan(nCases,1);

RiseResult = nan(nCases,1);
OvershootResult = nan(nCases,1);

TurnrateRMSE = nan(nCases,1);

HeadingError = nan(nCases,1);
HeadingIAE = nan(nCases,1);

MaxVoltage = nan(nCases,1);
VoltageRMS = nan(nCases,1);

VoltageSaturationPct = nan(nCases,1);
TurnrateSaturationPct = nan(nCases,1);

StableLinear = false(nCases,1);
StableDiscrete = false(nCases,1);

%% 11. GRID SEARCH

fprintf('============================================\n');
fprintf(' GRID SEARCH\n');
fprintf('============================================\n');

fprintf('Testing %d combinations...\n\n',nCases);

caseNumber = 0;

% Discrete approximation of the simplified turnrate plant
GturnZ = c2d(Gturn,Ts,'zoh');

for i = 1:nKp

    for j = 1:nKi

        caseNumber = caseNumber+1;

        KpTest = KpValues(i);
        KiTest = KiValues(j);

        KpResult(caseNumber) = KpTest;
        KiResult(caseNumber) = KiTest;

        %% 11.1 Linear continuous-time analysis

        Ctest = KpTest + KiTest/s;

        Ltest = minreal(Ctest*Gturn);
        Ttest = minreal(feedback(Ltest,1));

        StableLinear(caseNumber) = isstable(Ttest);

        if ~StableLinear(caseNumber)
            continue;
        end

        BWtest = bandwidth(Ttest);

        [~,PMtest] = margin(Ltest);

        BWResult(caseNumber) = BWtest;

        if isfinite(BWtest) && BWtest>0
            NResult(caseNumber) = BWinner/BWtest;
        end

        PMResult(caseNumber) = PMtest;

        %% 11.2 Discrete-time linear stability

        CtestZ = c2d(Ctest,Ts,'tustin');

        TtestZ = feedback(CtestZ*GturnZ,1);

        StableDiscrete(caseNumber) = isstable(TtestZ);

        if ~StableDiscrete(caseNumber)
            continue;
        end

        %% 11.3 Nonlinear discrete simulation

        sim = simulateBasebotTurnrate( ...
            KpTest,KiTest,p,t,vReference,wReference);

        omega = sim.turnrate;

        %% 11.4 Turnrate RMSE

        e = wReference(stepMask)-omega(stepMask);

        TurnrateRMSE(caseNumber) = sqrt(mean(e.^2));

        %% 11.5 Heading error

        eHeading = thetaReference-sim.heading;

        % Signed heading error at end of step

        HeadingError(caseNumber) = ...
            eHeading(idxStepEnd)*180/pi;

        % Integral absolute heading error
        % over first 0.5 seconds after step

        HeadingIAE(caseNumber) = trapz( ...
            t(transientMask), ...
            abs(eHeading(transientMask)));

        %% 11.6 Rise time

        tStep = t(stepMask)-stepStart;
        wStep = omega(stepMask);

        idx10 = find(wStep>=0.05,1,'first');
        idx90 = find(wStep>=0.45,1,'first');

        if ~isempty(idx10) && ~isempty(idx90)

            RiseResult(caseNumber) = ...
                tStep(idx90)-tStep(idx10);

        end

        %% 11.7 Overshoot

        OvershootResult(caseNumber) = ...
            max(0,(max(wStep)-0.5)/0.5*100);

        %% 11.8 Control effort

        voltage = sim.voltage;

        MaxVoltage(caseNumber) = max(abs(voltage(:)));

        VoltageRMS(caseNumber) = ...
            sqrt(mean(voltage(:).^2));

        VoltageSaturationPct(caseNumber) = ...
            100*mean(abs(voltage(:))>=Vmax-1e-6);

        TurnrateSaturationPct(caseNumber) = ...
            100*mean( ...
                abs(sim.turnrateCommand(stepMask)) ...
                >=turnrateMax-1e-6);

    end

end

fprintf('Grid search completed.\n\n');

%% 12. RESULTS TABLE

Results = table( ...
    KpResult, ...
    KiResult, ...
    BWResult, ...
    NResult, ...
    PMResult, ...
    RiseResult, ...
    OvershootResult, ...
    TurnrateRMSE, ...
    HeadingError, ...
    HeadingIAE, ...
    MaxVoltage, ...
    VoltageRMS, ...
    VoltageSaturationPct, ...
    TurnrateSaturationPct, ...
    StableLinear, ...
    StableDiscrete, ...
    'VariableNames',{ ...
    'Kp', ...
    'Ki', ...
    'BW_rad_s', ...
    'InnerOuterRatio', ...
    'PhaseMargin_deg', ...
    'RiseTime_s', ...
    'Overshoot_pct', ...
    'TurnrateRMSE', ...
    'HeadingError_deg', ...
    'HeadingIAE', ...
    'MaxVoltage_V', ...
    'VoltageRMS_V', ...
    'VoltageSaturation_pct', ...
    'TurnrateSaturation_pct', ...
    'StableLinear', ...
    'StableDiscrete'});

%% 13. ACCEPTANCE CRITERIA

% No imposed bandwidth separation.
%
% Screening thresholds:
%   Phase margin >= 45 degrees
%   Overshoot <= 20%
%   Turnrate command saturation <= 5%
%   Voltage saturation <= 5%
%
% These are explicit screening criteria, not
% universal guarantees of good performance.

PMmin = 45;
OvershootMax = 20;
TurnrateSatMax = 5;
VoltageSatMax = 5;

Valid = ...
    Results.StableLinear & ...
    Results.StableDiscrete & ...
    Results.PhaseMargin_deg>=PMmin & ...
    Results.Overshoot_pct<=OvershootMax & ...
    Results.TurnrateSaturation_pct<=TurnrateSatMax & ...
    Results.VoltageSaturation_pct<=VoltageSatMax & ...
    isfinite(Results.HeadingIAE);

ValidResults = Results(Valid,:);

% Primary objective: minimize heading IAE
% Secondary objective: minimize turnrate RMSE

ValidResults = sortrows( ...
    ValidResults, ...
    {'HeadingIAE','TurnrateRMSE'}, ...
    {'ascend','ascend'});

%% 14. PRINT BEST CANDIDATES

fprintf('============================================\n');
fprintf(' SEARCH RESULTS\n');
fprintf('============================================\n');

fprintf('Total combinations: %d\n',height(Results));
fprintf('Valid combinations: %d\n\n',height(ValidResults));

hasBest = ~isempty(ValidResults);

if hasBest

    KpBest = ValidResults.Kp(1);
    KiBest = ValidResults.Ki(1);

    fprintf('BEST CANDIDATE\n');

    fprintf('Kp = %.8f\n',KpBest);
    fprintf('Ki = %.8f\n',KiBest);
    fprintf('tau_i = %.8f s\n',KpBest/KiBest);

    fprintf('Outer BW = %.6f rad/s\n', ...
        ValidResults.BW_rad_s(1));

    fprintf('Inner/outer ratio = %.4f:1\n', ...
        ValidResults.InnerOuterRatio(1));

    fprintf('Phase margin = %.4f deg\n', ...
        ValidResults.PhaseMargin_deg(1));

    fprintf('Heading IAE = %.8f rad*s\n\n', ...
        ValidResults.HeadingIAE(1));

    fprintf('TOP 10 CANDIDATES\n');

    disp(ValidResults( ...
        1:min(10,height(ValidResults)),:));

else

    fprintf('No candidate satisfies all criteria.\n');
    fprintf('Inspect Results before changing thresholds.\n');

end

%% 15. SIMULATE REFERENCE CONTROLLERS

simOriginal = simulateBasebotTurnrate( ...
    KpOriginal,KiOriginal,p,t,vReference,wReference);

simFast = simulateBasebotTurnrate( ...
    KpFast,KiFast,p,t,vReference,wReference);

if hasBest

    simBest = simulateBasebotTurnrate( ...
        KpBest,KiBest,p,t,vReference,wReference);

end

%% 16. ORIGINAL CONTROLLER RESULTS

idxOriginal = ...
    abs(Results.Kp-KpOriginal)<1e-9 & ...
    abs(Results.Ki-KiOriginal)<1e-9;

fprintf('============================================\n');
fprintf(' ORIGINAL CONTROLLER\n');
fprintf('============================================\n');

disp(Results(idxOriginal,:));

%% 17. REFERENCE BANDWIDTH COMPARISON

Coriginal = KpOriginal + KiOriginal/s;
Cfast = KpFast + KiFast/s;

Toriginal = minreal(feedback(Coriginal*Gturn,1));
Tfast = minreal(feedback(Cfast*Gturn,1));

BWoriginal = bandwidth(Toriginal);
BWfast = bandwidth(Tfast);

fprintf('============================================\n');
fprintf(' BANDWIDTH COMPARISON\n');
fprintf('============================================\n');

fprintf('Inner BW: %.4f rad/s\n',BWinner);

fprintf('Original outer BW: %.4f rad/s\n',BWoriginal);
fprintf('Original ratio N: %.4f:1\n',BWinner/BWoriginal);

fprintf('Fast outer BW: %.4f rad/s\n',BWfast);
fprintf('Fast ratio N: %.4f:1\n',BWinner/BWfast);

if hasBest

    Cbest = KpBest + KiBest/s;
    Tbest = minreal(feedback(Cbest*Gturn,1));

    BWbest = bandwidth(Tbest);

    fprintf('Best outer BW: %.4f rad/s\n',BWbest);
    fprintf('Best ratio N: %.4f:1\n',BWinner/BWbest);

end

fprintf('\n');

%% 18. TURNRATE RESPONSE PLOT

figure('Name','Turnrate comparison');

plot(t,wReference,'k--','LineWidth',1.5);
hold on;

plot(t,simOriginal.turnrate,'LineWidth',1.3);
plot(t,simFast.turnrate,'LineWidth',1.3);

legendNames = { ...
    'Reference', ...
    'Original Basebot', ...
    'Fast 3:1'};

if hasBest

    plot(t,simBest.turnrate,'LineWidth',1.3);

    legendNames{end+1} = 'Best candidate';

end

grid on;
xlim([1.8 4.2]);

xlabel('Time (s)');
ylabel('Turnrate (rad/s)');
title('Turnrate step response');

legend(legendNames,'Location','best');

%% 19. HEADING ERROR PLOT

figure('Name','Heading error comparison');

plot(t, ...
    (thetaReference-simOriginal.heading)*180/pi, ...
    'LineWidth',1.3);

hold on;

plot(t, ...
    (thetaReference-simFast.heading)*180/pi, ...
    'LineWidth',1.3);

headingLegend = { ...
    'Original Basebot', ...
    'Fast 3:1'};

if hasBest

    plot(t, ...
        (thetaReference-simBest.heading)*180/pi, ...
        'LineWidth',1.3);

    headingLegend{end+1} = 'Best candidate';

end

yline(0,'k--');

grid on;
xlim([1.8 4.2]);

xlabel('Time (s)');
ylabel('Heading error (deg)');
title('Accumulated heading error');

legend(headingLegend,'Location','best');

%% 20. MOTOR VOLTAGES

figure('Name','Motor voltages - original');

plot(t,simOriginal.voltage(:,1),'LineWidth',1.1);
hold on;

plot(t,simOriginal.voltage(:,2),'LineWidth',1.1);

yline(Vmax,'k--');
yline(-Vmax,'k--');

grid on;
xlim([1.8 4.2]);

xlabel('Time (s)');
ylabel('Motor voltage (V)');
title('Original Basebot - motor voltages');

legend('Left','Right','Location','best');

%% 21. BANDWIDTH RATIO MAP

Nmap = reshape(Results.InnerOuterRatio,nKi,nKp);

figure('Name','Bandwidth ratio map');

imagesc(KpValues,KiValues,Nmap);

set(gca,'YDir','normal');
colorbar;

xlabel('Kp');
ylabel('Ki');

title('N = Inner bandwidth / Outer bandwidth');

hold on;

plot(KpOriginal,KiOriginal,'wx', ...
    'MarkerSize',12,'LineWidth',2);

if hasBest

    plot(KpBest,KiBest,'wo', ...
        'MarkerSize',10,'LineWidth',2);

end

%% 22. HEADING ERROR MAP

IAEmap = reshape(Results.HeadingIAE,nKi,nKp);

figure('Name','Heading error map');

imagesc(KpValues,KiValues,IAEmap);

set(gca,'YDir','normal');
colorbar;

xlabel('Kp');
ylabel('Ki');

title('Heading IAE during first 0.5 s');

hold on;

plot(KpOriginal,KiOriginal,'wx', ...
    'MarkerSize',12,'LineWidth',2);

if hasBest

    plot(KpBest,KiBest,'wo', ...
        'MarkerSize',10,'LineWidth',2);

end

%% 23. SAVE RESULTS

writetable(Results,'turnrate_neighborhood_all.csv');

if hasBest

    writetable(ValidResults, ...
        'turnrate_neighborhood_ranked.csv');

end

fprintf('CSV files saved.\n');

%% ============================================================
% LOCAL FUNCTION: DISCRETE-TIME NONLINEAR ROBOT SIMULATION
% ============================================================

function out = simulateBasebotTurnrate( ...
    KpTurn,KiTurn,p,t,vReference,wReference)

    n = numel(t);
    Ts = p.Ts;

    %% Motor states

    % Both motors use forward-positive coordinates.
    %
    % x = [current; angular velocity]

    current = zeros(2,1);
    motorOmega = zeros(2,1);

    %% Inner wheel PI integrators

    wheelIntegral = zeros(2,1);
    wheelPreviousError = zeros(2,1);

    %% Outer turnrate PI integrator

    turnIntegral = 0;
    turnPreviousError = 0;

    %% Output arrays

    turnrate = zeros(n,1);
    heading = zeros(n,1);

    voltage = zeros(n,2);
    turnrateCommand = zeros(n,1);

    wheelVelocity = zeros(n,2);

    %% Initial heading

    theta = 0;

    %% Simulation loop

    for k = 1:n

        %% 1. Measured wheel velocities

        vWheel = motorOmega*p.wheelRadius/p.gear;

        vLeft = vWheel(1);
        vRight = vWheel(2);

        measuredTurnrate = ...
            (vRight-vLeft)/p.wheelBase;

        turnrate(k) = measuredTurnrate;
        heading(k) = theta;

        wheelVelocity(k,:) = vWheel';

        %% 2. Outer turnrate PI

        eTurn = wReference(k)-measuredTurnrate;

        oldIntegral = turnIntegral;

        newIntegral = turnIntegral + ...
            0.5*Ts*(eTurn+turnPreviousError);

        command = KpTurn*eTurn + KiTurn*newIntegral;

        integralChange = ...
            KiTurn*(newIntegral-oldIntegral);

        % Conditional anti-windup

        if (command>p.turnrateMax && integralChange>0) || ...
           (command< -p.turnrateMax && integralChange<0)

            newIntegral = oldIntegral;

            command = KpTurn*eTurn + KiTurn*newIntegral;

        end

        turnIntegral = newIntegral;
        turnPreviousError = eTurn;

        command = max(-p.turnrateMax, ...
                      min(p.turnrateMax,command));

        turnrateCommand(k) = command;

        %% 3. Wheel velocity references

        vLeftRef = vReference(k) ...
            -0.5*p.wheelBase*command;

        vRightRef = vReference(k) ...
            +0.5*p.wheelBase*command;

        omegaRef = [vLeftRef;vRightRef] ...
            *p.gear/p.wheelRadius;

        %% 4. Inner wheel PI controllers

        eWheel = omegaRef-motorOmega;

        u = zeros(2,1);

        for m = 1:2

            oldIntegral = wheelIntegral(m);

            newIntegral = oldIntegral + ...
                0.5*Ts*(eWheel(m)+wheelPreviousError(m));

            voltageCommand = ...
                p.KpWheel*eWheel(m) ...
                +p.KiWheel*newIntegral;

            integralChange = ...
                p.KiWheel*(newIntegral-oldIntegral);

            if (voltageCommand>p.Vmax && integralChange>0) || ...
               (voltageCommand< -p.Vmax && integralChange<0)

                newIntegral = oldIntegral;

                voltageCommand = ...
                    p.KpWheel*eWheel(m) ...
                    +p.KiWheel*newIntegral;

            end

            wheelIntegral(m) = newIntegral;
            wheelPreviousError(m) = eWheel(m);

            u(m) = max(-p.Vmax, ...
                       min(p.Vmax,voltageCommand));

        end

        voltage(k,:) = u';

        %% 5. Motor dynamics - exact ZOH

        % Discrete motor matrices are calculated
        % once in the main script and passed in p.
        %
        % This avoids persistent variables and
        % repeated discretization.

        for m = 1:2

            x = [current(m);motorOmega(m)];

            xNext = p.Ad*x + p.Bd*u(m);

            current(m) = xNext(1);
            motorOmega(m) = xNext(2);

        end

        %% 6. Integrate robot heading

        if k<n

            theta = theta + measuredTurnrate*Ts;

        end

    end

    %% Return simulation results

    out.turnrate = turnrate;
    out.heading = heading;

    out.voltage = voltage;
    out.turnrateCommand = turnrateCommand;

    out.wheelVelocity = wheelVelocity;

end