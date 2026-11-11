%% BASEBOT - Ensaio em linha reta
% Missão: 0 -> 0.75 m/s em 2 s, depois 0.75 -> 0 m/s em 2 s.
% Compatível com o printLog() original de 29 colunas.
% Atualização: retas e percurso completo; 29, 30 ou mais colunas.
% Os comentários originais acima descrevem o ensaio original.

clear;
clc;
close all;

%% 1. Parâmetro usado neste ensaio

mu = 0.2;                 % ALTERAR para o valor usado no C++
g = 9.81;                  % [m/s^2]
accMax = mu * g;          % Limite configurado [m/s^2]

% Valores de recurso: o cabeçalho do log tem prioridade quando disponível.
wheelBase = 0.147;
voltageMax = 9.0;          % [V] voltageMax no C++; não é a tensão da bateria
circleRadius = 0.5;
straightSeparation = 0.3;
entryLength = 1.0;
exitLength = 1.5;
accelSmoothSeconds = 0.025; % suavização SÓ das velocidades medidas para derivar

%% 2. Ler o log — compatível com 29, 30 ou mais colunas

[file, folder] = uigetfile( ...
    {'*.log;*.txt;*.csv', 'Basebot logs (*.log, *.txt, *.csv)'}, ...
    'Seleciona o log do Basebot');

if isequal(file, 0)
    return;
end

lines = splitlines(string(fileread(fullfile(folder, file))));
rows = cell(numel(lines), 1);
rowLine = zeros(numel(lines), 1);
n = 0;

for k = 1:numel(lines)
    line = strtrim(lines(k));

    % Ignorar cabeçalhos, comentários e linhas vazias.
    if strlength(line) == 0 || startsWith(line, "%")
        continue;
    end

    values = sscanf(char(strrep(line, ',', ' ')), '%f');

    % As primeiras 29 colunas contêm todos os sinais necessários.
    if numel(values) >= 29 && all(isfinite(values(1:29)))
        n = n + 1;
        rows{n} = values.';
        rowLine(n) = k;
    end
end

if n < 2
    error('O ficheiro não contém pelo menos duas amostras com 29 ou mais colunas.');
end

rows = rows(1:n);
rowLine = rowLine(1:n);

% Preservar todas as colunas disponíveis.
% Se houver linhas com menos colunas, preencher as restantes com NaN.
nColumns = max(cellfun(@numel, rows));
data = NaN(n, nColumns);
for k = 1:n
    data(k, 1:numel(rows{k})) = rows{k};
end

% Se o monitor imprimiu o log várias vezes, usar o último bloco.
% Detetar também um novo cabeçalho entre duas linhas numéricas.
headers = find(contains(lines, '% BASEBOT -'));
newBlock = diff(data(:,1)) <= 0;
for k = 2:n
    newBlock(k-1) = newBlock(k-1) || ...
        any(headers > rowLine(k-1) & headers < rowLine(k));
end
lastReset = find(newBlock, 1, 'last');
firstRow = 1;
if ~isempty(lastReset)
    firstRow = lastReset + 1;
end
firstNumericLine = rowLine(firstRow);
lastNumericLine = rowLine(end);
data = data(firstRow:end, :);

% Usar apenas o cabeçalho correspondente ao último bloco selecionado.
previousNumericLine = 0;
if firstRow > 1
    previousNumericLine = rowLine(firstRow-1);
end
headerText = strjoin(lines(previousNumericLine+1:firstNumericLine-1), newline);
missionToken = regexp(char(headerText), 'Selected mission:\s*([^\r\n]+)', 'tokens', 'once');
missionName = 'Missão não identificada no cabeçalho';
if ~isempty(missionToken)
    missionName = strtrim(missionToken{1});
end
muManual = mu;
mu = headerNumber(headerText, 'miu configured in reference limiter', mu);
accMax = headerNumber(headerText, 'Reference acceleration limit [m/s^2]', mu*g);
wheelBase = headerNumber(headerText, 'Wheel base [m]', wheelBase);
voltageMax = headerNumber(headerText, 'Motor voltage limit [V]', voltageMax);
assert(mu > 0 && accMax > 0 && wheelBase > 0 && voltageMax > 0, ...
    'Os parâmetros de limites/geometria têm de ser positivos.');
fprintf('Log importado: %d amostras e %d colunas.\n', size(data,1), size(data,2));
if mu ~= muManual
    fprintf('mu atualizado pelo cabeçalho: %.3f (valor manual: %.3f).\n', mu, muManual);
end

%% 3. Extrair sinais

t = data(:, 1);
t = t - t(1);

state       = data(:, 2);
vRef        = data(:, 3);       % Referência ideal pedida pela missão
motorVLeft  = data(:, 4);
motorVRight = data(:, 5);
turnrateRef = data(:, 16);      % saída do controlador de heading
vEnc        = data(:, 17);      % Velocidade de avanço pelos encoders
turnrateEnc = data(:, 18);
xEnc        = data(:, 19);
yEnc        = data(:, 20);
headingEnc  = data(:, 21);      % [rad]
vWheelLeft  = data(:, 24);
vWheelRight = data(:, 25);
vRefLeft    = data(:, 26);      % Referência da roda esquerda [m/s]
vRefRight   = data(:, 27);      % Referência da roda direita [m/s]
headingRef  = data(:, 28);      % heading efetivamente pedido pela missão
headingGyro = data(:, 29);      % Heading já integrado no C++ [rad]
trajectoryTurnrateRef = NaN(size(t));
if size(data,2) >= 30
    trajectoryTurnrateRef = data(:,30);
end

if numel(t) < 2 || any(diff(t) <= 0)
    error('Os tempos do ensaio têm de ser estritamente crescentes.');
end

isFullPath = strcmp(missionName, 'sequenceFullPath') || ...
    any(ismember(state, [40 41 42 43 44 45 47]));
if isFullPath
    % Full-path mission
    if ~all(ismember([41 43 45], unique(state)))
        warning('Log parcial: não aparecem todas as fases da trajetória completa.');
    end
    active = ismember(state, [40 41 42 43 44 45 47]);
else
    % Standalone straight mission
    if ~any(state == 10) || ~any(state == 11)
        warning('O log pode não conter aceleração e travagem completas da missão em reta.');
    end
    active = ismember(state, [10 11 12 13 14 15 30 31 32 33]);
end
if ~any(active)
    error('Não foram identificadas amostras de controlo ativo.');
end
% O estado 12 ainda usa os PI para travar.
% No estado 2, o firmware impõe tensão e referências das rodas a zero.

%% 4. Reconstruir a referência de avanço após o limitador

% Mixer do C++:
%   vRefLeft  = vLimited - vTurnLimited
%   vRefRight = vLimited + vTurnLimited
vLimited = 0.5 * (vRefLeft + vRefRight);
vTurnLimited = 0.5 * (vRefRight - vRefLeft); % B/2 * omega, em m/s

% Diferenças apenas entre duas amostras consecutivas com controlo ativo.
% Exclui a transição para o estado 2.
dt = diff(t);
validPair = active(1:end-1) & active(2:end);
if ~any(validPair)
    error('Amostras insuficientes para calcular a aceleração.');
end

tMid = 0.5 * (t(1:end-1) + t(2:end));
aRefAll = diff(vLimited) ./ dt;
aTurnRefAll = diff(vTurnLimited) ./ dt;
aWheelRefAll = diff([vRefLeft vRefRight]) ./ dt;

% As derivadas medidas são ruidosas; suavizar velocidades antes de derivar.
smoothSamples = max(1, round(accelSmoothSeconds/median(dt)));
vMeasuredSmooth = movmean([vEnc, wheelBase/2*turnrateEnc, ...
    vWheelLeft, vWheelRight], smoothSamples, 1);
aMeasuredAll = diff(vMeasuredSmooth) ./ dt;

tAcc = tMid(validPair);
aRef = aRefAll(validPair);
aTurnRef = aTurnRefAll(validPair);
aWheelRef = aWheelRefAll(validPair,:);
aMeasured = aMeasuredAll(validPair,:);

peakAccel = max([0; aRef]);     % Pico positivo [m/s^2]
peakBrake = min([0; aRef]);     % Pico negativo [m/s^2]
peakAbs   = max(abs(aRef));     % Maior módulo [m/s^2]

%% 5. Reconstruir as três trajetórias

% 1) Ideal: integrar a referência registada, com heading desejado de 0 rad.
% Atualização: usar a coluna 28; heading zero só descreve uma reta sem correção.
% Esta é a trajetória cinemática dos comandos, não a geometria da fita.
xIdeal = cumtrapz(t, vRef .* cos(headingRef));
yIdeal = cumtrapz(t, vRef .* sin(headingRef));

% 2) Heading do gyro + velocidade de avanço dos encoders.
xGyro = cumtrapz(t, vEnc .* cos(headingGyro));
yGyro = cumtrapz(t, vEnc .* sin(headingGyro));

% 3) Odometria dos encoders, já calculada no C++.
xEnc = xEnc - xEnc(1);
yEnc = yEnc - yEnc(1);

% Erro de posição em relação à posição ideal no mesmo instante.
errGyro = hypot(xGyro - xIdeal, yGyro - yIdeal);
errEnc  = hypot(xEnc  - xIdeal, yEnc  - yIdeal);

% Geometria marcada: eixos do C++, X para a frente no início, Y para a esquerda.
if isFullPath
    phi0 = -pi/2 + asin(straightSeparation/(2*circleRadius));
    arcAngle = 2*pi - 2*asin(straightSeparation/(2*circleRadius));
    phi = linspace(phi0, phi0+arcAngle, 1200).';
    tapeX = [linspace(0,entryLength,300).'; ...
        entryLength+circleRadius*(sin(phi)-sin(phi0)); ...
        linspace(entryLength,entryLength-exitLength,300).'];
    tapeY = [zeros(300,1); straightSeparation/2-circleRadius*cos(phi); ...
        straightSeparation*ones(300,1)];
else
    targetDistance = headerNumber(headerText, 'Distance [m]', NaN);
    if ~isfinite(targetDistance)
        targetDistance = max(xIdeal);
    end
    tapeX = [0; targetDistance];
    tapeY = [0; 0];
end

% Erros de tracking sem wrap: o firmware usa heading contínuo.
vError = vEnc-vRef;
headingErrorGyro = headingGyro-headingRef;
headingErrorEnc = headingEnc-headingRef;

%% 6. Prints

fprintf('\n========== BASEBOT: ENSAIO EM RETA ==========\n');
% Título histórico preservado; a linha seguinte identifica a missão real.
fprintf('Missão: %s | percurso completo: %d\n', missionName, isFullPath);
fprintf('Log: %s\n', file);
fprintf('Amostras: %d | Duração registada: %.3f s\n', numel(t), t(end));

fprintf('\n--- ACELERAÇÃO DA REFERÊNCIA DE AVANÇO ---\n');
fprintf('mu usado no C++ (introduzido manualmente): %.2f\n', muManual);
fprintf('mu efetivo (cabeçalho, se disponível):      %.3f\n', mu);
fprintf('Limite configurado, mu*g:                 %.4f m/s^2\n', accMax);
fprintf('Pico de aceleração da referência:          %+.4f m/s^2\n', peakAccel);
fprintf('Pico de travagem da referência:            %+.4f m/s^2\n', peakBrake);
fprintf('Máximo absoluto observado:                  %.4f m/s^2\n', peakAbs);
fprintf('Pico observado / limite configurado:        %.1f %%\n', 100*peakAbs/accMax);
fprintf('Pico absoluto da componente de rotação:     %.4f m/s^2\n', max(abs(aTurnRef)));
fprintf('Picos absolutos das referências E/D:        %.4f / %.4f m/s^2\n', ...
    max(abs(aWheelRef(:,1))), max(abs(aWheelRef(:,2))));
fprintf('Os limitadores de avanço e B/2*omega estão ANTES do mixer.\n');
fprintf('mu*g não é um limite individual garantido para cada roda nesta implementação.\n');

fprintf('\n--- VELOCIDADE MÁXIMA ---\n');
fprintf('Referência ideal da missão:       %.4f m/s\n', max(vRef));
fprintf('Referência após o limitador:      %.4f m/s\n', max(vLimited(active)));
fprintf('Velocidade estimada por encoders: %.4f m/s\n', max(vEnc));
fprintf('Pico |V| motores E/D:             %.3f / %.3f V\n', ...
    max(abs(motorVLeft)), max(abs(motorVRight)));
fprintf('RMSE velocidade (controlo ativo): %.4f m/s\n', sqrt(mean(vError(active).^2)));
fprintf('RMSE heading gyro (ativo):        %.3f deg\n', ...
    rad2deg(sqrt(mean(headingErrorGyro(active).^2))));

fprintf('\n--- TRAJETÓRIA: POSIÇÃO FINAL ---\n');
fprintf('%-27s %10s %10s %14s\n', 'Trajetória', 'X [m]', 'Y [m]', 'Erro XY [m]');
fprintf('%-27s %10.4f %10.4f %14.4f\n', 'Referência integrada', xIdeal(end), yIdeal(end), 0);
fprintf('%-27s %10.4f %10.4f %14.4f\n', ...
    'Gyro + velocidade encoder', xGyro(end), yGyro(end), errGyro(end));
fprintf('%-27s %10.4f %10.4f %14.4f\n', ...
    'Odometria dos encoders', xEnc(end), yEnc(end), errEnc(end));
fprintf('Fim da geometria marcada: X = %.4f m; Y = %.4f m\n', tapeX(end), tapeY(end));

fprintf('\n--- DESVIO DURANTE O PERCURSO ---\n');
if ~isFullPath
    fprintf('Máximo |Y|, gyro + velocidade encoder: %.2f cm\n', 100*max(abs(yGyro)));
    fprintf('Máximo |Y|, odometria dos encoders:    %.2f cm\n', 100*max(abs(yEnc)));
else
    fprintf('No percurso completo, |Y| não representa desvio lateral à fita.\n');
end
fprintf('Heading final, gyro:                   %+.2f deg\n', rad2deg(headingGyro(end)));
fprintf('Heading final, encoders:               %+.2f deg\n', rad2deg(headingEnc(end)));
fprintf('=============================================\n');

%% 7. Figura 1: velocidade e aceleração

cGyro = [0.00 0.38 0.75];
cEnc = [0.85 0.30 0.12];
cRef = [0.15 0.15 0.15];
vLimitedPlot = vLimited;
vLimitedPlot(~active) = NaN;
figure('Color','w','Name','Referências e respostas','Position',[80 60 1100 820]);
axTracking = gobjects(4,1);
axTracking(1) = subplot(4,1,1);
plot(t,vRef,'--','Color',cRef,'LineWidth',1.7); hold on;
plot(t,vLimitedPlot,'Color',[0.3 0.65 0.3],'LineWidth',1.3);
plot(t,vEnc,'Color',cGyro,'LineWidth',1.7);
grid on; ylabel('Velocidade [m/s]');
title(sprintf('%s | mu = %.3f', missionName, mu),'Interpreter','none');
legend('Referência da missão','Referência após limitador','Medida: encoders','Location','best');
axTracking(2) = subplot(4,1,2);
plot(t,rad2deg(headingRef),'--','Color',cRef,'LineWidth',1.7); hold on;
plot(t,rad2deg(headingEnc),'Color',cEnc,'LineWidth',1.0);
plot(t,rad2deg(headingGyro),'Color',cGyro,'LineWidth',2.2);
grid on; ylabel('Heading [deg]'); title('Referência e headings contínuos medidos');
legend('Referência','Encoders','Gyro','Location','best');
axTracking(3) = subplot(4,1,3);
plot(t,vError,'Color',cGyro,'LineWidth',1.3); hold on; yline(0,'k:');
grid on; ylabel('Erro v [m/s]'); title('Velocidade medida menos referência da missão');
axTracking(4) = subplot(4,1,4);
plot(t,rad2deg(headingErrorEnc),'Color',cEnc,'LineWidth',1.0); hold on;
plot(t,rad2deg(headingErrorGyro),'Color',cGyro,'LineWidth',1.8); yline(0,'k:');
grid on; xlabel('Tempo [s]'); ylabel('Erro heading [deg]');
legend('Encoders - referência','Gyro - referência','Location','best');
linkaxes(axTracking,'x');

% Gráficos separados de aceleração para não misturar os limites físicos e de referência.
figure('Color','w','Name','Acelerações e limites','Position',[110 65 1100 850]);
axAccel = gobjects(4,1);
axAccel(1) = subplot(4,1,1);
plot(tAcc,aMeasured(:,1),'Color',[0.65 0.65 0.65],'LineWidth',0.9); hold on;
plot(tAcc,aRef,'Color',cGyro,'LineWidth',1.6);
yline(accMax,'r--','+\mu g','HandleVisibility','off');
yline(-accMax,'r--','-\mu g','HandleVisibility','off'); yline(0,'k:','HandleVisibility','off');
grid on; ylabel('a avanço [m/s^2]');
title('Avanço: limite imposto à referência antes do mixer');
legend('Medida: derivada suavizada','Referência limitada','Location','best');
axAccel(2) = subplot(4,1,2);
plot(tAcc,aMeasured(:,2),'Color',[0.65 0.65 0.65],'LineWidth',0.9); hold on;
plot(tAcc,aTurnRef,'Color',cGyro,'LineWidth',1.6);
yline(accMax,'r--','+\mu g','HandleVisibility','off');
yline(-accMax,'r--','-\mu g','HandleVisibility','off');
grid on; ylabel('d(B\omega/2)/dt [m/s^2]');
title('Rotação: limite imposto à componente B/2 vezes omega antes do mixer');
legend('Medida: encoders, suavizada','Referência limitada','Location','best');
for j = 1:2
    axAccel(j+2) = subplot(4,1,j+2);
    plot(tAcc,aMeasured(:,j+2),'Color',[0.65 0.65 0.65],'LineWidth',0.9); hold on;
    plot(tAcc,aWheelRef(:,j),'Color',cGyro,'LineWidth',1.6);
    yline(accMax,'r--','+\mu g (comparação)','HandleVisibility','off');
    yline(-accMax,'r--','-\mu g (comparação)','HandleVisibility','off');
    grid on; ylabel('a roda [m/s^2]');
    sides = {'esquerda','direita'};
    title(sprintf('Roda %s: soma das componentes; mu*g não é imposto aqui', sides{j}));
    legend('Medida: derivada suavizada','Referência após mixer','Location','best');
end
xlabel('Tempo [s]'); linkaxes(axAccel,'x');
fprintf('Suavização das velocidades para acelerações medidas: %d amostras (~%.1f ms).\n', ...
    smoothSamples,1000*smoothSamples*median(dt));

%% 8. Figura 2: trajetória ideal vs gyro vs encoders

figure('Color','w','Name','Comparação das trajetórias','Position',[140 80 1050 820]);
subplot(2,1,1);
% Geometria larga e clara em fundo; gyro desenhado por último e mais espesso.
plot(tapeX,tapeY,'-','Color',[0.80 0.80 0.80],'LineWidth',7); hold on;
plot(xIdeal,yIdeal,'k--','LineWidth',1.3);
plot(xEnc,yEnc,'Color',cEnc,'LineWidth',1.1);
plot(xGyro,yGyro,'Color',cGyro,'LineWidth',2.8);
plot(0,0,'go','MarkerFaceColor','g','HandleVisibility','off');
plot(tapeX(end),tapeY(end),'ks','MarkerFaceColor','k','HandleVisibility','off');
plot(xGyro(end),yGyro(end),'o','Color',cGyro,'MarkerFaceColor',cGyro,'HandleVisibility','off');
plot(xEnc(end),yEnc(end),'x','Color',cEnc,'LineWidth',1.5,'HandleVisibility','off');
axis equal; grid on; xlabel('X [m]'); ylabel('Y [m]');
title('Trajetória no chão: fita em fundo, gyro em destaque');
legend('Percurso marcado','Referência integrada (v e heading)', ...
    'Odometria dos encoders','Gyro + velocidade dos encoders','Location','best');

% Mostra desvios de poucos centímetros, difíceis de ver em axis equal.
subplot(2,1,2);
if isFullPath
    plot(t,100*errEnc,'Color',cEnc,'LineWidth',1.1); hold on;
    plot(t,100*errGyro,'Color',cGyro,'LineWidth',2.3);
    ylabel('Erro XY [cm]');
    title('Distância à referência integrada no mesmo instante (não distância à fita)');
    legend('Encoders','Gyro + velocidade dos encoders','Location','best');
else
    plot(t,100*yGyro,'Color',cGyro,'LineWidth',2.3); hold on;
    plot(t,100*yEnc,'Color',cEnc,'LineWidth',1.1); yline(0,'k--');
    ylabel('Desvio lateral Y [cm]');
    title('Desvio lateral estimado em relação à reta ideal');
    legend('Gyro + velocidade dos encoders','Odometria dos encoders', ...
        'Ideal','Location','best');
end
grid on; xlabel('Tempo [s]');

%% 9. Tensões aplicadas aos motores
figure('Color','w','Name','Inputs dos motores em V','Position',[170 100 1000 650]);
axVoltage = gobjects(2,1);
for j = 1:2
    axVoltage(j) = subplot(2,1,j);
    volts = [motorVLeft motorVRight];
    plot(t,volts(:,j),'Color',cGyro,'LineWidth',1.3); hold on;
    yline(voltageMax,'r--','+V_{max}'); yline(-voltageMax,'r--','-V_{max}');
    yline(0,'k:'); grid on; ylabel('Tensão [V]');
    sides = {'esquerdo','direito'};
    title(sprintf('Motor %s | convenção de sinal do log C++',sides{j}));
end
xlabel('Tempo [s]'); linkaxes(axVoltage,'x');

%% 10. Referências e velocidades de cada roda
figure('Color','w','Name','Tracking das rodas','Position',[200 120 1000 650]);
axWheel = gobjects(2,1);
for j = 1:2
    axWheel(j) = subplot(2,1,j);
    refs = [vRefLeft vRefRight]; measures = [vWheelLeft vWheelRight];
    plot(t,refs(:,j),'k--','LineWidth',1.5); hold on;
    plot(t,measures(:,j),'Color',cGyro,'LineWidth',1.5);
    grid on; ylabel('Velocidade [m/s]');
    sides = {'esquerda','direita'}; title(sprintf('Roda %s',sides{j}));
    legend('Referência após mixer','Medida: encoder','Location','best');
end
xlabel('Tempo [s]'); linkaxes(axWheel,'x');

%% 11. Turn rate: feedback e futura referência de feedforward
figure('Color','w','Name','Turn rate','Position',[230 140 1000 430]);
plot(t,turnrateRef,'k--','LineWidth',1.3); hold on;
plot(t,turnrateEnc,'Color',cEnc,'LineWidth',1.0);
labels = {'Referência do controlador de heading (col. 16)','Medida: encoders'};
if any(isfinite(trajectoryTurnrateRef))
    plot(t,trajectoryTurnrateRef,'Color',cGyro,'LineWidth',2);
    labels{end+1} = 'Referência da trajetória (col. 30)';
end
grid on; xlabel('Tempo [s]'); ylabel('Turn rate [rad/s]');
title('A coluna 30 é independente da saída do controlador de heading');
legend(labels,'Location','best');

%travagem menor: device-monitor-261111-113354
%travagem maior: device-monitor-261111-114312 APPROACH ESCOLHIDA

% Sem turnrate controller
%1.0m em casa (correção de heading nula): device-monitor-261111-160025 - desvio maximo 0.45cm
%1.5m em casa (correção de heading nula): device-monitor-261111-161530 - desvio maximo 3.08cm

% Com turnrate controller
%1.0m em casa (correção de heading nula): device-monitor-261111-162638 - desvio maximo 1.08cm
%1.5m em casa (correção de heading nula): device-monitor-261111-163921 - desvio maximo 1.50 cm

% (Nota!: A velocidade mais alta nos 1.5m está a deixar a performance muito imprevisível, 
% temos de perceber se isso também acontece na sala, se acontecer não vale a pena isto para poupar 0.1s, 
% mantemos a approach anterior de apenas 1.5 m/s de velocidade máxima com travagem menor)


% Trajetória Completa

% Sem turnrate controller
% para já nada de jeito

% Com turnrate controller
% para já nada de jeito

%% Função local: ler parâmetros numéricos do cabeçalho
function value = headerNumber(text, key, fallback)
    pattern = [regexptranslate('escape',key), ...
        '\s*=\s*([-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?)'];
    tokens = regexp(char(text),pattern,'tokens');
    value = fallback;
    if ~isempty(tokens)
        parsed = str2double(tokens{end}{1});
        if isfinite(parsed)
            value = parsed;
        end
    end
end
