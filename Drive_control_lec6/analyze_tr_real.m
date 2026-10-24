%% Basebot - real turnrate PI experiment (Lecture 7)
% Run once for the conservative PI and again for the fast PI.
% Keep the same test sequence and use the same analysis for both runs.
clear; close all; clc;

logFile = 'device-monitor-261024-163207.log';
controllerName = 'Fast 3:1';
KpTurn = 0.0443322593;
tauiTurn = 0.00778527218;
KiTurn = KpTurn/tauiTurn;
voltageLimit = 9;

%% Read the 27-column serial log, ignoring comments and USB messages
fid = fopen(logFile,'r');
assert(fid~=-1,'Cannot open log file: %s',logFile);
rows = zeros(0,27);
while ~feof(fid)
    line = strtrim(fgetl(fid));
    if isempty(line) || startsWith(line,'%'), continue; end
    values = sscanf(line,'%f').';
    if numel(values)==27 && all(isfinite(values))
        rows(end+1,:) = values; %#ok<SAGROW>
    end
end
fclose(fid);
assert(~isempty(rows),'No valid 27-column log rows found.');
% The L command can print the same saved run multiple times.
% Keep the LAST complete occurrence of each time stamp.
[~,keep] = unique(rows(:,1),'last');
rows = sortrows(rows(sort(keep),:),1);

t = rows(:,1);
vRef = rows(:,3);
uL = rows(:,4); uR = rows(:,5);
iL = rows(:,10); iR = rows(:,11);
battery = rows(:,12);
wRef = rows(:,16); v = rows(:,17); w = rows(:,18);
x = rows(:,19); y = rows(:,20); heading = rows(:,21);
signedDistance = rows(:,22); totalDistance = rows(:,23);
vL = rows(:,24); vR = rows(:,25);
vLRef = rows(:,26); vRRef = rows(:,27);

% Detect the turn step from the logged reference (no assumed start time).
stepIndex = find(abs(diff(wRef))>0.1 & abs(wRef(2:end))>0.1,1,'first')+1;
assert(~isempty(stepIndex),'No turnrate step found in column 16.');
t0 = t(stepIndex);
endIndex = find(t>t0 & abs(wRef)<0.1,1,'first');
if isempty(endIndex), endIndex=numel(t)+1; end
turn = stepIndex:endIndex-1;
tTurn = t(turn)-t0;
r = median(wRef(turn));
assert(abs(r)>0,'Zero turnrate reference.');

% Evaluate baseline and the last 20%% of the turn interval.
pre = t>=max(t(1),t0-0.4) & t<t0;
tailStart = t0 + 0.8*(t(turn(end))-t0);
tail = t>=tailStart & t<=t(turn(end));
baselineW = mean(w(pre));
meanW = mean(w(tail));
steadyError = r-meanW;
rmseW = sqrt(mean((w(turn)-wRef(turn)).^2));
maeW = mean(abs(w(turn)-wRef(turn)));
peakW = max(sign(r)*w(turn));
overshoot = max(0,(peakW-abs(r))/abs(r))*100;

% 10-90%% rise time relative to the pre-step measured baseline.
normalized = (w(turn)-baselineW)/(r-baselineW);
i10 = find(normalized>=0.1,1,'first');
i90 = find(normalized>=0.9,1,'first');
if ~isempty(i10) && ~isempty(i90) && i90>=i10
    riseTime=tTurn(i90)-tTurn(i10);
else
    riseTime=NaN;
end

% 2%% settling: all subsequent RAW samples must remain inside the band.
% If the robot never settles before the reference changes, return NaN.
lastOutside=find(abs(w(turn)-r)>0.02*abs(r),1,'last');
if isempty(lastOutside)
    settlingTime=0;
elseif lastOutside==numel(turn)
    settlingTime=NaN;
else
    settlingTime=tTurn(lastOutside+1);
end

% Compare the wheel PI performance and actuator effort in the turn interval.
wheelRMSEL=sqrt(mean((vL(turn)-vLRef(turn)).^2));
wheelRMSER=sqrt(mean((vR(turn)-vRRef(turn)).^2));
forwardRMSE=sqrt(mean((v(turn)-vRef(turn)).^2));
maxVoltage=max(abs([uL(turn);uR(turn)]));
saturationPercent=100*mean(abs(uL(turn))>=voltageLimit-0.05 | ...
                           abs(uR(turn))>=voltageLimit-0.05);
voltageRMS=sqrt(mean((uL(turn).^2+uR(turn).^2)/2));

% Heading change: compare actual integrated heading with reference integral.
headingChange=heading(turn(end))-heading(turn(1));
expectedHeading=trapz(t(turn),wRef(turn));
headingError=expectedHeading-headingChange;

fprintf('\n=== %s ===\n',controllerName);
fprintf('KpTurn %.8f | KiTurn %.8f | tauiTurn %.8f s\n',KpTurn,KiTurn,tauiTurn);
fprintf('Turn step: t=%.3f s | reference=%.3f rad/s | duration=%.3f s\n', ...
    t0,r,tTurn(end));
fprintf('Straight-line baseline turnrate: %.4f rad/s\n',baselineW);
fprintf('Rise time (10-90%%, raw): %.4f s\n',riseTime);
fprintf('Overshoot (raw): %.2f %%\n',overshoot);
fprintf('Settling time (2%%, raw): %.4f s (NaN = not settled)\n',settlingTime);
fprintf('Final mean turnrate: %.4f rad/s | steady error: %.4f rad/s\n',meanW,steadyError);
fprintf('Turnrate RMSE: %.4f rad/s | MAE: %.4f rad/s\n',rmseW,maeW);
fprintf('Forward velocity RMSE: %.4f m/s\n',forwardRMSE);
fprintf('Wheel velocity RMSE: left %.4f, right %.4f m/s\n',wheelRMSEL,wheelRMSER);
fprintf('Motor voltage: peak %.3f V | RMS %.3f V | saturation %.2f %%\n', ...
    maxVoltage,voltageRMS,saturationPercent);
fprintf('Heading change: actual %.4f rad | expected %.4f rad | error %.4f rad\n', ...
    headingChange,expectedHeading,headingError);
fprintf('Final position: x=%.3f m, y=%.3f m | signed distance=%.3f m\n', ...
    x(end),y(end),signedDistance(end));

%% Plots: raw measurements + optional smoothing ONLY for visualization
smoothN = max(3,round(0.05/median(diff(t)))); % 50 ms moving mean
figure('Color','w','Name','Turnrate and forward velocity');
tiledlayout(3,1);
nexttile; hold on; grid on;
plot(t,w,'Color',[0.65 0.75 0.95],'DisplayName','Measured (raw)');
plot(t,movmean(w,smoothN),'b','LineWidth',1.4,'DisplayName','Measured (50 ms average)');
plot(t,wRef,'k--','LineWidth',1.3,'DisplayName','Reference');
xline(t0,':'); ylabel('Turnrate (rad/s)'); legend('Location','best');
nexttile; hold on; grid on;
plot(t,v,'b','DisplayName','Measured'); plot(t,vRef,'k--','DisplayName','Reference');
ylabel('Forward velocity (m/s)'); legend('Location','best');
nexttile; hold on; grid on;
plot(t,wRef-w,'DisplayName','Turnrate error');
yline(0,'k:'); xline(t0,':'); ylabel('Error (rad/s)'); xlabel('Time (s)');

figure('Color','w','Name','Wheels and motor voltages');
tiledlayout(3,1);
nexttile; hold on; grid on;
plot(t,vL,'b',t,vLRef,'b--',t,vR,'r',t,vRRef,'r--');
ylabel('Wheel speed (m/s)'); legend('Left','Left ref','Right','Right ref','Location','best');
nexttile; hold on; grid on;
plot(t,uL,'b',t,uR,'r');
yline(voltageLimit,'k:'); yline(-voltageLimit,'k:');
ylabel('Motor voltage (V)'); legend('Left','Right','Location','best');
nexttile; hold on; grid on;
plot(t,iL,'b',t,iR,'r'); ylabel('Motor current (A)');
xlabel('Time (s)'); legend('Left','Right','Location','best');

figure('Color','w','Name','Robot motion');
tiledlayout(2,2);
nexttile; plot(x,y,'b','LineWidth',1.5); hold on; grid on; axis equal;
plot(x(1),y(1),'go',x(end),y(end),'rx');
xlabel('x (m)'); ylabel('y (m)'); title('Measured trajectory');
nexttile; plot(t,heading,'LineWidth',1.2); grid on;
xlabel('Time (s)'); ylabel('Heading (rad)');
nexttile; plot(t,signedDistance,t,totalDistance); grid on;
xlabel('Time (s)'); ylabel('Distance (m)');
legend('Signed','Total','Location','best');
nexttile; plot(t,battery); grid on;
xlabel('Time (s)'); ylabel('Battery (V)');

%% Save comparable numerical results (run the fast test with another name)
result = struct('name',controllerName,'KpTurn',KpTurn,'KiTurn',KiTurn, ...
    'tauiTurn',tauiTurn,'stepTime',t0,'reference',r, ...
    'riseTime',riseTime,'overshoot',overshoot,'settlingTime',settlingTime, ...
    'steadyError',steadyError,'rmseW',rmseW,'maeW',maeW, ...
    'forwardRMSE',forwardRMSE,'wheelRMSEL',wheelRMSEL,'wheelRMSER',wheelRMSER, ...
    'maxVoltage',maxVoltage,'voltageRMS',voltageRMS, ...
    'saturationPercent',saturationPercent,'headingError',headingError, ...
    'tRelative',tTurn,'wMeasured',w(turn),'wReference',wRef(turn), ...
    'x',x,'y',y,'t',t);
outputFile = sprintf('turnrate_results_%s.mat',regexprep(controllerName,'[^A-Za-z0-9]','_'));
save(outputFile,'result');
fprintf('Saved %s\n',outputFile);

%% After both experiments, run this in MATLAB to compare them:
% a=load('turnrate_results_Conservative_4_1.mat');
% b=load('turnrate_results_Fast_3_1.mat');
% A=a.result; B=b.result;
% metrics={'riseTime','overshoot','settlingTime','steadyError','rmseW', ...
%          'forwardRMSE','wheelRMSEL','wheelRMSER','voltageRMS', ...
%          'saturationPercent','headingError'};
% comparison=table(string(metrics'), ...
%    cellfun(@(s) A.(s),metrics)',cellfun(@(s) B.(s),metrics)', ...
%    'VariableNames',{'Metric','Conservative','Fast'});
% disp(comparison);
% figure; hold on; grid on;
% plot(A.tRelative,A.wMeasured,'DisplayName','Conservative 4:1');
% plot(B.tRelative,B.wMeasured,'DisplayName','Fast 3:1');
% plot(A.tRelative,A.wReference,'k--','DisplayName','Reference');
% xlabel('Time since turn step (s)'); ylabel('Turnrate (rad/s)');
% legend('Location','best');
