%% LEONORA_FAST_MISSION - DO WE REALLY NEED SUCH A COMPLEX CODE FOR THIS?? (EXPLORE...)
% Run this script to create v_input and w_input for your CURRENT Simulink model.
% w_input contains HEADING [rad], despite its existing name.
% No controller changes or extra toolboxes are required.
%
% Default: fixed timed stops and heading steps at both joins.
% Optional old mode searches a family of smooth joins, then computes a
% fast rest-to-rest timing subject to both wheel acceleration constraints.
% This is a numerical optimum within the searched path family and grid;
% it is not a global optimum including motor dynamics or tracking errors.
%
% The tolerance constrains the NOMINAL path. Actual tracking error adds to it.
% mu is an empirical parameter in aWheelMax = mu*g, not a full tyre model.
% Reference: https://arxiv.org/abs/1707.07239 (reachability-based time scaling).

clc
clearvars
close all

%% PARAMETERS YOU CAN CHANGE
useTimedHeadingWaits = true; % true: timed stops/heading steps; false: old smooth planner
T_wait_heading = 0.60;      % [s] FIXED wait at EACH join; not scaled by slowFactor
vCircleMax = 0.50;          % [m/s] circle speed cap BEFORE division by slowFactor
lineTolerance = 0.05;       % [m] nominal maximum line deviation: 0.02 or 0.05
mu = 0.35;                 % [-] change for the floor: e.g. 0.30 or 0.40
g = 9.81;                  % [m/s^2]
aWheelMax = mu*g;          % [m/s^2] limit on EACH wheel, acceleration/braking
vWheelMax = Inf;           % [m/s] no wheel speed restriction for now
aLatMax = 1000;            % [m/s^2] effectively inactive for this mission
B = 0.147;                 % [m] wheel spacing

% Geometry assumed from the slide. Measure from the ACTUAL mission start/stop.
circleRadius = 0.50;       % [m] 1 m circle diameter
straightSeparation = 0.30; % [m] distance between straight centre lines
entryLength = 1.00;        % [m] start to circle join; check timing mark position
exitLength = 1.50;         % [m] circle join to stop; check stop mark position

t_start = 0.50;            % [s] stationary time before starting
t_hold = 0.50;             % [s] stationary time after finishing
dt_ref = 0.01;             % [s] From Workspace reference sampling interval

% Numerical settings. Increase nJoinCandidates to search joins more finely.
nJoinCandidates = 15;      % candidates PER join (entry/exit searched separately)
ds_search = 0.010;         % [m] coarse path grid used during join search
ds_final = 0.002;          % [m] final grid; automatically refined if needed
makePlots = true;
slowFactor = 1.3;          % >=1: slows movement; in timed-wait mode, waits stay exactly T_wait_heading

%% CHECK SETTINGS AND DEFINE THE MARKED ROUTE
assert(lineTolerance > 0, 'lineTolerance must be positive.');
assert(T_wait_heading > 0 && isfinite(T_wait_heading));
assert(vCircleMax > 0);
assert(mu > 0 && isfinite(mu), 'mu must be finite and positive.');
assert(aWheelMax > 0 && isfinite(aWheelMax), 'aWheelMax must be finite and positive.');
assert(vWheelMax > 0 && aLatMax > 0, 'Speed/lateral limits must be positive.');
assert(B > 0 && circleRadius > straightSeparation/2 && straightSeparation > 0);
assert(entryLength > 0 && exitLength > 0 && dt_ref > 0);
assert(t_start >= 0 && t_hold >= 0 && nJoinCandidates >= 2);
assert(ds_search > 0 && ds_final > 0);
assert(isscalar(slowFactor) && isfinite(slowFactor) && slowFactor >= 1, ...
    'slowFactor must be a finite scalar >= 1.');

geom.R = circleRadius;
geom.d = straightSeparation;
geom.Lin = entryLength;
geom.Lout = exitLength;
geom.phi0 = -pi/2 + asin(geom.d/(2*geom.R));
geom.arcAngle = 2*pi - 2*asin(geom.d/(2*geom.R));
geom.Larc = geom.R*geom.arcAngle;
geom.yJoin = -sqrt(geom.R^2 - (geom.d/2)^2);
geom.total = geom.Lin + geom.Larc + geom.Lout;

limits.B = B;
limits.aWheel = aWheelMax;
limits.vWheel = vWheelMax;
limits.aLat = aLatMax;

%% TIMED MISSION WITH FIXED HEADING WAITS
if useTimedHeadingWaits
    % Entire schedule is calculated OFFLINE. No state machine or measured
    % heading/velocity is used to decide when a segment starts.
    % Each moving segment accelerates and brakes to zero reference speed.
    % At each join, heading steps to the next direction; v_ref stays zero
    % for exactly T_wait_heading, then the next moving segment starts.
    % Actual turn-in-place motion is produced by the heading/turn-rate loops;
    % its wheel accelerations are not predicted by this reference generator.

    wheelFactor = 1 + B/(2*circleRadius);
    circleSpeedCap = min([vCircleMax, vWheelMax/wheelFactor, ...
                          sqrt(aLatMax*circleRadius)]);
    straightProfile = fixedMotionProfile(entryLength,aWheelMax,vWheelMax,slowFactor);
    circleProfile = fixedMotionProfile(geom.Larc,aWheelMax/wheelFactor,circleSpeedCap,slowFactor);
    exitProfile = fixedMotionProfile(exitLength,aWheelMax,vWheelMax,slowFactor);

    t_entry_start = t_start;
    t_entry_end = t_entry_start + straightProfile.T;
    t_circle_start = t_entry_end + T_wait_heading;
    t_circle_end = t_circle_start + circleProfile.T;
    t_exit_start = t_circle_end + T_wait_heading;
    t_stop = t_exit_start + exitProfile.T;
    Tsim = t_stop + t_hold;

    % Include both sides of heading steps so linear From Workspace
    % interpolation represents a step rather than a dt_ref-long ramp.
    stepEpsilon = min(1e-6,dt_ref/100);
    events = [t_entry_start; t_entry_end-stepEpsilon; t_entry_end; ...
              t_circle_start; t_circle_end-stepEpsilon; t_circle_end; ...
              t_exit_start; t_stop; Tsim];
    % Include acceleration/cruise/braking breakpoints for exact speed sampling.
    knots = [t_entry_start+[straightProfile.ta; straightProfile.ta+straightProfile.tc]; ...
             t_circle_start+[circleProfile.ta; circleProfile.ta+circleProfile.tc]; ...
             t_exit_start+[exitProfile.ta; exitProfile.ta+exitProfile.tc]];
    t_ref = unique([(0:dt_ref:Tsim)'; events; knots]);
    [sEntry,vEntry] = evaluateFixedMotion(t_ref-t_entry_start,straightProfile);
    [sCircle,vCircle] = evaluateFixedMotion(t_ref-t_circle_start,circleProfile);
    [sExit,vExit] = evaluateFixedMotion(t_ref-t_exit_start,exitProfile);
    velocity_ref = vEntry + vCircle + vExit;
    s_ref = sEntry + sCircle + sExit;

    % Coordinates: initial heading 0 along +x, +y to the robot's left.
    x_ref = sEntry; y_ref = zeros(size(t_ref));
    onCircle = t_ref >= t_entry_end & t_ref < t_circle_end;
    phi = geom.phi0 + sCircle(onCircle)/circleRadius;
    x_ref(onCircle) = circleRadius*sin(phi)-(geom.yJoin-entryLength);
    y_ref(onCircle) = straightSeparation/2-circleRadius*cos(phi);
    afterCircle = t_ref >= t_circle_end;
    x_ref(afterCircle) = entryLength-sExit(afterCircle);
    y_ref(afterCircle) = straightSeparation;

    heading_ref = zeros(size(t_ref));
    heading_ref(onCircle) = phi;  % fixed during first wait, varies on circle
    heading_ref(afterCircle) = pi; % second step, then fixed during exit
    % Moving-circle turn rate only. Heading steps are executed by feedback.
    % Leave the trajectory feedforward branch disconnected in this mode.
    turnrate_ref = vCircle/circleRadius;
    v_input = [t_ref velocity_ref];
    w_input = [t_ref heading_ref];
    turnrate_input = [t_ref turnrate_ref];

    wait1 = t_ref>=t_entry_end & t_ref<=t_circle_start;
    wait2 = t_ref>=t_circle_end & t_ref<=t_exit_start;
    assert(all(abs(velocity_ref(wait1 | wait2))<1e-10), ...
        'Velocity reference must be zero throughout both heading waits.');
    assert(abs(t_circle_start-t_entry_end-T_wait_heading)<1e-10);
    assert(abs(t_exit_start-t_circle_end-T_wait_heading)<1e-10);
    assert(abs(s_ref(end)-geom.total)<1e-9 && abs(velocity_ref(end))<1e-10);
    assert(abs(x_ref(end)-(entryLength-exitLength))<1e-9);
    assert(abs(y_ref(end)-straightSeparation)<1e-9);

    mission.mode = 'fixed timed heading waits';
    mission.parameters.slowFactor = slowFactor;
    mission.parameters.T_wait_heading = T_wait_heading;
    mission.parameters.mu = mu;
    mission.parameters.lineTolerance = lineTolerance;
    mission.parameters.vCircleMax = vCircleMax;
    mission.limits = limits;
    mission.motionTime = t_stop-t_start;
    mission.waitIntervals = [t_entry_end t_circle_start; t_circle_end t_exit_start];
    mission.segmentIntervals = [t_entry_start t_entry_end; ...
                               t_circle_start t_circle_end; t_exit_start t_stop];
    fprintf('Timed mission: %.3f s (includes two %.2f s heading waits)\n', ...
            mission.motionTime,T_wait_heading);
    fprintf('Wait before circle: %.3f to %.3f s\n',t_entry_end,t_circle_start);
    fprintf('Wait before exit:   %.3f to %.3f s\n',t_circle_end,t_exit_start);
    fprintf('Circle peak reference speed: %.3f m/s\n',circleProfile.vPeak);
    fprintf('Set Simulink Stop time to Tsim = %.3f s\n',Tsim);

    if makePlots
        [tapeXY,~,~] = markedRoute(linspace(0,geom.total,3001)',geom);
        tapeXY = toStartFrame(tapeXY,geom);
        figure('Name','Leonora timed route');
        plot(tapeXY(:,1),tapeXY(:,2),'k--','LineWidth',1.5); hold on;
        plot(x_ref,y_ref,'b','LineWidth',1.5);
        plot(x_ref(1),y_ref(1),'go','MarkerFaceColor','g');
        plot(x_ref(end),y_ref(end),'ro','MarkerFaceColor','r');
        axis equal; grid on; xlabel('x [m]'); ylabel('y [m]');
        legend('Marked route','Nominal path','Start','Stop','Location','best');
        title('Timed route with heading changes at rest');
        figure('Name','Timed mission references');
        subplot(3,1,1); plot(t_ref,velocity_ref,'LineWidth',1.5); grid on;
        ylabel('v_{ref} [m/s]'); title('Two fixed heading waits');
        subplot(3,1,2); plot(t_ref,heading_ref,'LineWidth',1.5); grid on;
        ylabel('Heading [rad]');
        subplot(3,1,3); plot(t_ref,turnrate_ref,'LineWidth',1.5); grid on;
        ylabel('Circle turn rate [rad/s]'); xlabel('Time [s]');
    end
else

%% FIND SMOOTH JOINS THAT RESPECT THE CHOSEN LINE TOLERANCE
% Each quintic join matches position, tangent and curvature at both ends.
% cut = distance removed on EACH side of the sharp marked join.
% Search extent is deliberately exposed here if you later broaden the family.
maxCut = min([0.45*entryLength, 0.45*exitLength, ...
              0.24*geom.Larc, 12*lineTolerance]);
minCut = min(0.002, maxCut/50);
cutTrials = linspace(minCut, maxCut, round(nJoinCandidates));
% Reserve only a numerical integration margin, not a tracking-error margin.
joinBudget = lineTolerance - min(1e-4,0.01*lineTolerance);
joinsIn = {}; joinsOut = {};
for jj = 1:numel(cutTrials)
    cut = cutTrials(jj);
    J = makeJoin(geom, geom.Lin, cut);
    J.errorBound = joinDeviation(J, geom, geom.Lin);
    if J.errorBound <= joinBudget
        joinsIn{end+1} = J; %#ok<SAGROW>
    end
    J = makeJoin(geom, geom.Lin + geom.Larc, cut);
    J.errorBound = joinDeviation(J, geom, geom.Lin + geom.Larc);
    if J.errorBound <= joinBudget
        joinsOut{end+1} = J; %#ok<SAGROW>
    end
end
assert(~isempty(joinsIn) && ~isempty(joinsOut), ...
    'No join met the tolerance; increase nJoinCandidates or reduce minCut.');
% Include a candidate near the tolerance boundary so the coarse cut grid
% does not leave a useful part of the available line tolerance unused.
joinsIn = refineJoinLimit(joinsIn,cutTrials,geom,geom.Lin,joinBudget);
joinsOut = refineJoinLimit(joinsOut,cutTrials,geom,geom.Lin+geom.Larc,joinBudget);

bestTime = Inf;
searchResults = zeros(numel(joinsIn)*numel(joinsOut), 3);
row = 0;
for ii = 1:numel(joinsIn)
    for jj = 1:numel(joinsOut)
        P = buildPath(geom, joinsIn{ii}, joinsOut{jj}, ds_search);
        timing = timeScale(P.s, P.kappa, limits);
        row = row + 1;
        searchResults(row,:) = [joinsIn{ii}.cut, joinsOut{jj}.cut, timing.T];
        if timing.T < bestTime
            bestTime = timing.T;
            bestIn = joinsIn{ii}; bestOut = joinsOut{jj};
        end
    end
end

%% REFINE THE SELECTED PATH AND COMPUTE ITS FINAL TIMING
% Integrating the sampled curvature must agree with the geometric tangent.
% This also checks that the sharpest part of a join is sufficiently resolved.
for refinement = 1:7
    path = buildPath(geom, bestIn, bestOut, ds_final);
    thetaIntegrated = path.theta(1) + cumtrapz(path.s, path.kappa);
    headingGridError = max(abs(thetaIntegrated - path.theta));
    if headingGridError <= 1e-5
        break
    end
    ds_final = ds_final/2;
end
assert(headingGridError <= 1e-5, ...
    'Curvature is under-resolved. Reduce ds_final or increase the refinement count.');
timing = timeScale(path.s, path.kappa, limits);

% Slow the TIMING MODEL before generating any sampled signals or diagnostics.
% This preserves the geometric path and scales all derivatives consistently.
nominalTiming = timing;
timing.t = slowFactor*nominalTiming.t;
timing.T = slowFactor*nominalTiming.T;
timing.speed2 = nominalTiming.speed2/slowFactor^2;
timing.pathAccel = nominalTiming.pathAccel/slowFactor^2;
timing.controllableSpeed2 = nominalTiming.controllableSpeed2/slowFactor^2;
t_start = slowFactor*t_start;
t_hold = slowFactor*t_hold;

% Reconstruct the position implied by the actual (integrated-curvature) refs.
% The coordinate frame has x forward at the start, y left, heading initially 0.
xNodes = cumtrapz(path.s, cos(thetaIntegrated));
yNodes = cumtrapz(path.s, sin(thetaIntegrated));
geometryGridError = max(hypot(xNodes-path.xy(:,1), yNodes-path.xy(:,2)));
plannedDeviationBound = max(bestIn.errorBound, bestOut.errorBound) + geometryGridError;
assert(plannedDeviationBound <= lineTolerance + 1e-9, ...
    'Refined path exceeds tolerance. Increase nJoinCandidates or reduce ds_final.');

%% CREATE THE SAME INPUT VARIABLES AS YOUR EXISTING SCRIPT


t_stop = t_start + timing.T;
Tsim = t_stop + t_hold;
% Include exact start, stop and end times in addition to the regular grid.
t_ref = unique([(0:dt_ref:Tsim)'; t_start; t_stop; Tsim]);
tau = min(max(t_ref - t_start, 0), timing.T);
idx = interp1(timing.t, (1:numel(timing.t))', tau, 'previous');
idx = min(max(idx,1), numel(path.s)-1);
localTime = tau - timing.t(idx);

% Exact constant-path-acceleration interpolation within each grid interval.
v0 = sqrt(timing.speed2(idx));
aPath = timing.pathAccel(idx);
velocity_ref = max(0, v0 + aPath.*localTime);
s_ref = path.s(idx) + v0.*localTime + 0.5*aPath.*localTime.^2;
s_ref = min(max(s_ref,0), path.s(end));
velocity_ref(t_ref <= t_start | t_ref >= t_stop) = 0;

dsLocal = s_ref - path.s(idx);
kappaSlope = diff(path.kappa)./diff(path.s);
kappa_ref = path.kappa(idx) + kappaSlope(idx).*dsLocal;
heading_ref = thetaIntegrated(idx) + path.kappa(idx).*dsLocal ...
            + 0.5*kappaSlope(idx).*dsLocal.^2;
turnrate_ref = velocity_ref.*kappa_ref;
x_ref = interp1(path.s, xNodes, s_ref, 'linear');
y_ref = interp1(path.s, yNodes, s_ref, 'linear');

v_input = [t_ref velocity_ref];
w_input = [t_ref heading_ref];       % heading, as in your existing model
turnrate_input = [t_ref turnrate_ref]; % optional future feedforward reference
wheel_left_ref = velocity_ref - B/2*turnrate_ref;
wheel_right_ref = velocity_ref + B/2*turnrate_ref;

% Diagnostic wheel accelerations in the same piecewise-linear curvature model.
aL = (1-B/2*kappa_ref).*aPath - B/2*kappaSlope(idx).*velocity_ref.^2;
aR = (1+B/2*kappa_ref).*aPath + B/2*kappaSlope(idx).*velocity_ref.^2;
stationary = t_ref < t_start | t_ref >= t_stop;
aL(stationary) = 0; aR(stationary) = 0;
assert(max(abs([aL; aR])) <= aWheelMax*(1+1e-7), ...
    'Sampled wheel acceleration exceeds aWheelMax.');
assert(max(abs([wheel_left_ref;wheel_right_ref])) <= vWheelMax*(1+1e-7), ...
    'Sampled wheel speed exceeds vWheelMax.');
assert(max(velocity_ref.^2.*abs(kappa_ref)) <= aLatMax*(1+1e-7), ...
    'Sampled lateral acceleration exceeds aLatMax.');

mission.path = path;
mission.timing = timing;
mission.nominalTiming = nominalTiming;
searchResults(:,3) = slowFactor*searchResults(:,3);
mission.searchResults = searchResults;
mission.joinCuts = [bestIn.cut bestOut.cut];
mission.nominalDeviationBound = plannedDeviationBound;
mission.geometryGridError = geometryGridError;
mission.limits = limits;
mission.parameters.lineTolerance = lineTolerance;
mission.parameters.mu = mu;
mission.parameters.slowFactor = slowFactor;
mission.endpointError = hypot(x_ref(end)-path.xy(end,1), y_ref(end)-path.xy(end,2));

fprintf('Motion time: %.3f s (excludes %.2f s pre-start hold)\n', timing.T, t_start);
fprintf('Nominal deviation bound: %.2f cm; chosen tolerance: %.2f cm\n', ...
    100*plannedDeviationBound, 100*lineTolerance);
fprintf('Wheel acceleration limit: %.3f m/s^2 (mu = %.3f)\n', aWheelMax, mu);
fprintf('Peak forward speed: %.3f m/s; peak wheel speed: %.3f m/s\n', ...
    max(velocity_ref), max(abs([wheel_left_ref;wheel_right_ref])));
fprintf('Peak lateral acceleration: %.3f m/s^2\n', ...
    max(velocity_ref.^2.*abs(kappa_ref)));
fprintf('Selected join cuts: %.3f m entry, %.3f m exit\n', bestIn.cut, bestOut.cut);
fprintf('Kinematic endpoint discretisation error: %.3f mm\n', 1000*mission.endpointError);

if makePlots
    [tapeXY,~,~] = markedRoute(linspace(0,geom.total,3001)', geom);
    tapeXY = toStartFrame(tapeXY, geom);
    figure('Name','Leonora planned route');
    plot(tapeXY(:,1), tapeXY(:,2), 'k--', 'LineWidth', 1.5); hold on;
    plot(x_ref, y_ref, 'b', 'LineWidth', 1.5);
    plot(x_ref(1), y_ref(1), 'go', 'MarkerFaceColor','g');
    plot(x_ref(end), y_ref(end), 'ro', 'MarkerFaceColor','r');
    axis equal; grid on; xlabel('x [m]'); ylabel('y [m]');
    legend('Marked route','Planned reference','Start','Stop','Location','best');
    title(sprintf('Nominal line tolerance %.1f cm; motion time %.2f s', ...
        100*lineTolerance, timing.T));

    figure('Name','Leonora mission references');
    subplot(3,2,1); plot(t_ref,velocity_ref); grid on;
    ylabel('v_{ref} [m/s]');
    subplot(3,2,2); plot(t_ref,heading_ref*180/pi); grid on;
    ylabel('Heading [deg]');
    subplot(3,2,3); plot(t_ref,turnrate_ref); grid on;
    ylabel('Turn rate [rad/s]');
    subplot(3,2,4); plot(t_ref,[wheel_left_ref wheel_right_ref]); grid on;
    ylabel('Wheel speed [m/s]'); legend('Left','Right');
    subplot(3,2,5); plot(t_ref,[aL aR]); hold on;
    plot([0 Tsim],[aWheelMax aWheelMax],'k--');
    plot([0 Tsim],[-aWheelMax -aWheelMax],'k--'); grid on;
    ylabel('Wheel acceleration [m/s^2]'); xlabel('Time [s]');
    subplot(3,2,6); plot(t_ref,velocity_ref.^2.*abs(kappa_ref)); grid on;
    ylabel('Lateral acceleration [m/s^2]'); xlabel('Time [s]');
end

end % useTimedHeadingWaits

%% LOCAL FUNCTIONS -- normally leave these unchanged
function [P,D,K] = markedRoute(s,G)
    s = s(:); P = zeros(numel(s),2); D = P; K = zeros(size(s));
    first = s <= G.Lin;
    last = s >= G.Lin + G.Larc;
    arc = ~first & ~last;
    P(first,:) = [G.d/2*ones(sum(first),1), G.yJoin-G.Lin+s(first)];
    D(first,2) = 1;
    phi = G.phi0 + (s(arc)-G.Lin)/G.R;
    P(arc,:) = G.R*[cos(phi),sin(phi)];
    D(arc,:) = [-sin(phi),cos(phi)]; K(arc) = 1/G.R;
    P(last,:) = [-G.d/2*ones(sum(last),1), ...
                 G.yJoin-(s(last)-G.Lin-G.Larc)];
    D(last,2) = -1;
end

function J = makeJoin(G, centre, cut)
    [P0,D0,K0] = markedRoute(centre-cut,G);
    [P1,D1,K1] = markedRoute(centre+cut,G);
    scale = 2*cut;
    C0 = P0; C1 = scale*D0;
    C2 = scale^2*K0*[-D0(2),D0(1)]/2;
    DD1 = scale^2*K1*[-D1(2),D1(1)];
    rhs = [P1-C0-C1-C2; scale*D1-C1-2*C2; DD1-2*C2];
    high = [1 1 1; 3 4 5; 6 12 20]\rhs;
    J.C = [C0; C1; C2; high]; J.cut = cut;
end

function [P,D,DD] = evaluateJoin(J,u)
    u = u(:); z = zeros(size(u)); o = ones(size(u));
    P = [o u u.^2 u.^3 u.^4 u.^5]*J.C;
    D = [z o 2*u 3*u.^2 4*u.^3 5*u.^4]*J.C;
    DD = [z z 2*o 6*u 12*u.^2 20*u.^3]*J.C;
end

function joins = refineJoinLimit(joins,cuts,G,centre,budget)
    lo = max(cellfun(@(J) J.cut,joins));
    higher = cuts(cuts>lo);
    if isempty(higher), return; end
    hi = min(higher);
    for kk = 1:12
        J = makeJoin(G,centre,(lo+hi)/2);
        J.errorBound = joinDeviation(J,G,centre);
        if J.errorBound<=budget
            lo = J.cut; best = J;
        else
            hi = J.cut;
        end
    end
    if exist('best','var'), joins{end+1} = best; end
end

function bound = joinDeviation(J,G,centre)
    % Two-way sampled deviation: also checks that the join covers the corner.
    u = linspace(0,1,801)';
    [P,D,~] = evaluateJoin(J,u);
    [Q,~,~] = markedRoute(linspace(centre-J.cut,centre+J.cut,801)',G);
    forward = distanceToMark(P,G);
    dist2 = (Q(:,1)-P(:,1)').^2 + (Q(:,2)-P(:,2)').^2;
    reverse = sqrt(min(dist2,[],2));
    % A small sampling margin avoids accepting a join just outside tolerance.
    margin = max(max(sqrt(sum(D.^2,2)))/800/2, J.cut/800);
    bound = max([forward;reverse]) + margin;
end

function d = distanceToMark(P,G)
    y1 = min(max(P(:,2),G.yJoin-G.Lin),G.yJoin);
    y2 = min(max(P(:,2),G.yJoin-G.Lout),G.yJoin);
    d1 = hypot(P(:,1)-G.d/2,P(:,2)-y1);
    d2 = hypot(P(:,1)+G.d/2,P(:,2)-y2);
    phase = mod(atan2(P(:,2),P(:,1))-G.phi0,2*pi);
    dc = abs(hypot(P(:,1),P(:,2))-G.R);
    inGap = phase > G.arcAngle;
    dc(inGap) = min(hypot(P(inGap,1)-G.d/2,P(inGap,2)-G.yJoin), ...
                    hypot(P(inGap,1)+G.d/2,P(inGap,2)-G.yJoin));
    d = min([d1 d2 dc],[],2);
end

function P = buildPath(G,Jin,Jout,ds)
    C = {routePiece(0,G.Lin-Jin.cut,G,ds), ...
         joinPiece(Jin,ds), ...
         routePiece(G.Lin+Jin.cut,G.Lin+G.Larc-Jout.cut,G,ds), ...
         joinPiece(Jout,ds), ...
         routePiece(G.Lin+G.Larc+Jout.cut,G.total,G,ds)};
    s = []; xy = []; D = []; K = []; offset = 0;
    for kk = 1:numel(C)
        piece = C{kk};
        keep = (1:numel(piece.s))';
        if kk > 1, keep = keep(2:end); end
        s = [s; offset+piece.s(keep)]; %#ok<AGROW>
        xy = [xy; piece.xy(keep,:)]; %#ok<AGROW>
        D = [D; piece.D(keep,:)]; %#ok<AGROW>
        K = [K; piece.K(keep)]; %#ok<AGROW>
        offset = offset+piece.s(end);
    end
    P.s = s; P.xy = toStartFrame(xy,G); P.kappa = K;
    P.theta = unwrap(atan2(-D(:,1),D(:,2)));
end

function C = routePiece(a,b,G,ds)
    ss = linspace(a,b,max(2,ceil((b-a)/ds)+1))';
    [C.xy,C.D,C.K] = markedRoute(ss,G); C.s = ss-a;
end

function C = joinPiece(J,ds)
    uu = linspace(0,1,max(1001,ceil(4*J.cut/ds)+1))';
    [~,D,~] = evaluateJoin(J,uu);
    speed = sqrt(sum(D.^2,2));
    assert(min(speed)>1e-10,'Degenerate join.');
    ss = cumtrapz(uu,speed);
    % Resolve every join even during coarse search: a very short join must
    % not appear artificially fast because its peak curvature was skipped.
    C.s = linspace(0,ss(end),max(101,ceil(ss(end)/ds)+1))';
    u = interp1(ss,uu,C.s,'pchip');
    [C.xy,C.D,DD] = evaluateJoin(J,u);
    C.K = (C.D(:,1).*DD(:,2)-C.D(:,2).*DD(:,1)) ...
           ./sum(C.D.^2,2).^(3/2);
end

function xy = toStartFrame(P,G)
    xy = [P(:,2)-(G.yJoin-G.Lin), G.d/2-P(:,1)];
end

function result = timeScale(s,kappa,L)
    % u=v^2. Each interval uses linear u(s) and linear kappa(s).
    % q_L/R=1-/+B*kappa/2 and a_wheel=q*u'/2+q'*u.
    % Wheel acceleration is linear in s within an interval, so enforcing BOTH
    % endpoints bounds the entire interval, even when an inner wheel reverses.
    ds = diff(s); N = numel(s);
    q = [1-L.B/2*kappa, 1+L.B/2*kappa];
    q0 = q(1:end-1,:); q1 = q(2:end,:);
    dq = (q1-q0)./ds;
    A0 = dq-q0./(2*ds); B0 = q0./(2*ds);
    A1 = -q1./(2*ds); B1 = dq+q1./(2*ds);
    A = [A0 -A0 A1 -A1];
    BB = [B0 -B0 B1 -B1];
    CC = L.aWheel*ones(N-1,8);

    cap = Inf(N,1);
    if isfinite(L.vWheel)
        cap = min(cap,(L.vWheel./max(abs(q),[],2)).^2);
    end
    if isfinite(L.aLat)
        curved = abs(kappa)>1e-14;
        cap(curved) = min(cap(curved),L.aLat./abs(kappa(curved)));
    end
    % Use the stricter cap at both ends of every interval. This conservatively
    % bounds speed/lateral acceleration BETWEEN the grid points as well.
    segmentCap = min(cap(1:end-1),cap(2:end));
    cap = min(cap,[segmentCap;Inf]);
    cap = min(cap,[Inf;segmentCap]);

    % Backward reachability: K(i) is largest u that can still reach rest.
    K = zeros(N,1);
    for ii = N-1:-1:1
        aa = [A(ii,:) 0 0];
        bb = [BB(ii,:) -1 1];
        cc = [CC(ii,:) 0 K(ii+1)];
        epsB = 1e-12*max(1,max(abs(bb)));
        upper = find(bb>epsB); lower = find(bb < -epsB);
        same = find(abs(bb)<=epsB);
        hi = cap(ii);
        for zz = same
            if aa(zz)>1e-12, hi = min(hi,cc(zz)/aa(zz)); end
        end
        for ll = lower
            coeff = aa(upper)./bb(upper)-aa(ll)/bb(ll);
            rhs = cc(upper)./bb(upper)-cc(ll)/bb(ll);
            active = coeff>1e-12;
            if any(active), hi = min(hi,min(rhs(active)./coeff(active))); end
        end
        assert(isfinite(hi) && hi>=-1e-8,'Unbounded/infeasible reachability interval.');
        K(ii) = max(0,hi);
    end

    % Forward pass: take the largest admissible next speed that remains
    % controllable, rather than accelerating too late to brake for a turn.
    u = zeros(N,1);
    for ii = 1:N-1
        aa = A(ii,:); bb = BB(ii,:); cc = CC(ii,:);
        epsB = 1e-12*max(1,max(abs(bb)));
        upper = bb>epsB; lower = bb < -epsB;
        hi = min(K(ii+1),min((cc(upper)-aa(upper)*u(ii))./bb(upper)));
        lo = max(0,max((cc(lower)-aa(lower)*u(ii))./bb(lower)));
        assert(hi>=lo-1e-7*max(1,lo),'Infeasible forward speed step.');
        u(ii+1) = max(0,hi);
    end
    v = sqrt(u);
    assert(all(v(1:end-1)+v(2:end)>0),'Zero-speed interval.');
    dt = 2*ds./(v(1:end-1)+v(2:end));
    violation = A.*u(1:end-1)+BB.*u(2:end)-CC;
    assert(max(violation(:))<=L.aWheel*1e-7,'Wheel acceleration check failed.');
    result.speed2 = u;
    result.t = [0;cumsum(dt)];
    result.T = result.t(end);
    result.pathAccel = diff(u)./(2*ds);
    result.controllableSpeed2 = K;
end

function P = fixedMotionProfile(distance,aMax,vMax,factor)
    % Rest-to-rest triangular/trapezoidal profile, then uniform time scaling.
    vp = min(sqrt(aMax*distance),vMax);
    ta = vp/aMax;
    tc = max(0,(distance-vp^2/aMax)/vp);
    P.distance = distance;
    P.a = aMax/factor^2;
    P.vPeak = vp/factor;
    P.ta = factor*ta;
    P.tc = factor*tc;
    P.T = 2*P.ta+P.tc;
end

function [s,v] = evaluateFixedMotion(t,P)
    s = zeros(size(t)); v = s;
    accelerating = t>=0 & t<P.ta;
    cruising = t>=P.ta & t<P.ta+P.tc;
    braking = t>=P.ta+P.tc & t<P.T;
    s(accelerating) = 0.5*P.a*t(accelerating).^2;
    v(accelerating) = P.a*t(accelerating);
    s(cruising) = 0.5*P.a*P.ta^2+P.vPeak*(t(cruising)-P.ta);
    v(cruising) = P.vPeak;
    remaining = P.T-t(braking);
    s(braking) = P.distance-0.5*P.a*remaining.^2;
    v(braking) = P.a*remaining;
    s(t>=P.T) = P.distance;
end
