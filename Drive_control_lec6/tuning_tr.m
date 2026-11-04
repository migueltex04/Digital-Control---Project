%% Compare outer turnrate PI controllers - verified against lec7 SLX
% Model inspection: From Workspace inputs v_input and w_input.
% Root To Workspace 'robot' is a 6-channel Timeseries:
% [v_ref, w_ref, distance, velocity, heading, turnrate].
% Root To Workspace 'pose1' and controller 'turn' also exist.
clear; close all; clc;

model='motor_model_23b_lec7';
L=3.57e-3; R=3.57; Kt=0.008863; Kb=Kt;
D=8.57e-7; J=0.94e-6+8.683e-6;
gear=9.6; wrad=0.0315*1.073; B=0.147; r2m=wrad/gear;
Kp=0.08; tau_I=0.4; Ki=Kp/tau_I;
Ts=0.0005;
Tend=3; stepTime=0.5; vCommand=0.6; rStep=1;
tvec=(0:Ts:Tend)';
v_input=timeseries(vCommand*ones(size(tvec)),tvec);
w_input=timeseries(rStep*double(tvec>=stepTime),tvec);

names={'Fast (3:1)','Conservative (4:1)'};
KpVals=[0.0443322593,0.0738802083];
KiVals=[5.694375,4.62734082];

load_system(model);
results=struct([]);
for k=1:2
    Kp_tr=KpVals(k); Ki_tr=KiVals(k); tau_I_tr=Kp_tr/Ki_tr;
    simIn=Simulink.SimulationInput(model);
    varNames={'L','R','Kt','Kb','D','J','gear','wrad','B','r2m', ...
        'Kp','Ki','tau_I','Ts','Kp_tr','Ki_tr','tau_I_tr','v_input','w_input'};
    for n=1:numel(varNames)
        simIn=simIn.setVariable(varNames{n},eval(varNames{n}));
    end
    simIn=simIn.setModelParameter('StopTime',num2str(Tend));
    fprintf('Running %s: Kp_tr=%.8f, Ki_tr=%.8f\n',names{k},Kp_tr,Ki_tr);
    simOut=sim(simIn);
    robotTS=simOut.get('robot');
    if ~isa(robotTS,'timeseries')
        error('Expected robot as timeseries; received %s',class(robotTS));
    end
    t=robotTS.Time(:);
    data=squeeze(robotTS.Data);
    if size(data,1)~=numel(t) && size(data,2)==numel(t), data=data.'; end
    if size(data,2)~=6
        error('Expected 6 robot channels; received %s',mat2str(size(data)));
    end
    % Motor voltage is already logged by the original SLX model.
    % Channel 5 of velP / velPR is voltage AFTER the +/-9 V saturation.
    leftTS=simOut.get('velP');
    rightTS=simOut.get('velPR');
    if ~isa(leftTS,'timeseries') || ~isa(rightTS,'timeseries')
        error('Expected velP and velPR as timeseries.');
    end
    vl=squeeze(leftTS.Data); vr=squeeze(rightTS.Data);
    if size(vl,1)~=numel(leftTS.Time), vl=vl.'; end
    if size(vr,1)~=numel(rightTS.Time), vr=vr.'; end
    results(k).tLeft=leftTS.Time(:);
    results(k).tRight=rightTS.Time(:);
    results(k).voltageLeft=vl(:,5);
    results(k).voltageRight=vr(:,5);
    results(k).peakLeft=max(abs(vl(:,5)));
    results(k).peakRight=max(abs(vr(:,5)));
    % Voltage exactly at the limit is consistent with clipping, but
    % cannot prove pre-saturation command exceeded the limit.
    tol=1e-4;
    results(k).limitLeft=sum(abs(vl(:,5))>=9-tol)/numel(leftTS.Time)*100;
    results(k).limitRight=sum(abs(vr(:,5))>=9-tol)/numel(rightTS.Time)*100;
    fprintf('  Left peak %.4f V (at limit %.2f%% samples) | Right peak %.4f V (at limit %.2f%% samples)\n', ...
        results(k).peakLeft,results(k).limitLeft,results(k).peakRight,results(k).limitRight);
    results(k).name=names{k};
    results(k).t=t;
    results(k).vref=data(:,1);
    results(k).wref=data(:,2);
    results(k).distance=data(:,3);
    results(k).velocity=data(:,4);
    results(k).heading=data(:,5);
    results(k).turnrate=data(:,6);
    after=t>=stepTime;
    ts=t(after)-stepTime;
    y=data(after,6);
    results(k).overshoot=max(0,(max(y)-rStep)/abs(rStep)*100);
    lastOutside=find(abs(y-rStep)>0.02*abs(rStep),1,'last');
    if isempty(lastOutside)
        results(k).settling=0;
    elseif lastOutside==numel(ts)
        results(k).settling=NaN;
    else
        results(k).settling=ts(lastOutside+1);
    end
    tail=t>=Tend-0.2;
    results(k).steadyError=rStep-mean(data(tail,6));
    results(k).speedError=vCommand-mean(data(tail,4));
    fprintf('  Overshoot %.3f%% | settling(2%%) %.4f s | final turnrate error %.5g rad/s\n', ...
        results(k).overshoot,results(k).settling,results(k).steadyError);
end

figure('Color','w','Name','Turnrate PI comparison');
subplot(3,1,1); hold on; grid on;
for k=1:2
    plot(results(k).t,results(k).turnrate,'LineWidth',1.2,'DisplayName',names{k});
end
plot(tvec,w_input.Data,'k--','DisplayName','Reference');
ylabel('Turnrate (rad/s)'); legend('Location','best');
subplot(3,1,2); hold on; grid on;
for k=1:2
    plot(results(k).t,results(k).velocity,'LineWidth',1.2,'DisplayName',names{k});
end
yline(vCommand,'k--','Reference'); ylabel('Velocity (m/s)');
subplot(3,1,3); hold on; grid on;
for k=1:2
    plot(results(k).t,results(k).heading,'LineWidth',1.2,'DisplayName',names{k});
end
xlabel('Time (s)'); ylabel('Heading (rad)');

fprintf('\n=== COMPARISON ===\n');
for k=1:2
    fprintf('%s: overshoot %.4f%% | settling %.4f s | turnrate error %.6f rad/s | velocity error %.6f m/s\n', ...
        names{k},results(k).overshoot,results(k).settling,results(k).steadyError,results(k).speedError);
end
fprintf('Voltage signals read from existing velP and velPR model outputs.\n');

figure('Color','w','Name','Motor voltages and saturation');
for k=1:2
    subplot(2,1,k); hold on; grid on;
    plot(results(k).tLeft,results(k).voltageLeft,'DisplayName','Left motor');
    plot(results(k).tRight,results(k).voltageRight,'DisplayName','Right motor');
    yline(9,'k--','+9 V limit'); yline(-9,'k--','-9 V limit');
    xline(stepTime,'k:','Turnrate step');
    ylabel('Voltage (V)'); title(names{k}); legend('Location','best');
end
xlabel('Time (s)');

fprintf('\n=== VOLTAGE SUMMARY (POST-SATURATION) ===\n');
for k=1:2
    fprintf('%s: left peak %.4f V, right peak %.4f V; limit hits left %.2f%%, right %.2f%%\n', ...
        names{k},results(k).peakLeft,results(k).peakRight,results(k).limitLeft,results(k).limitRight);
end
