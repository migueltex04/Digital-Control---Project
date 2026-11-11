%% BASEBOT - Ensaio em linha reta
% Missão: 0 -> 0.75 m/s em 2 s, depois 0.75 -> 0 m/s em 2 s.
% Compatível com o printLog() original de 29 colunas.

clear;
clc;
close all;

%% 1. Parâmetro usado neste ensaio

mu = 0.2;                 % ALTERAR para o valor usado no C++
g = 9.81;                  % [m/s^2]
accMax = mu * g;          % Limite configurado [m/s^2]

%% 2. Ler o log — compatível com 29, 30 ou mais colunas

[file, folder] = uigetfile( ...
    {'*.log;*.txt;*.csv', 'Basebot logs (*.log, *.txt, *.csv)'}, ...
    'Seleciona o log da missão em linha reta');

if isequal(file, 0)
    return;
end

lines = splitlines(string(fileread(fullfile(folder, file))));

rows = cell(numel(lines), 1);
n = 0;

for k = 1:numel(lines)
    line = strtrim(lines(k));

    % Ignorar cabeçalhos, comentários e linhas vazias.
    if strlength(line) == 0 || startsWith(line, "%")
        continue;
    end

    values = sscanf(char(line), '%f');

    % As primeiras 29 colunas contêm todos os sinais necessários.
    if numel(values) >= 29
        n = n + 1;
        rows{n} = values.';
    end
end

if n < 2
    error('O ficheiro não contém pelo menos duas amostras com 29 ou mais colunas.');
end

rows = rows(1:n);

% Preservar todas as colunas disponíveis.
% Se houver linhas com menos colunas, preencher as restantes com NaN.
nColumns = max(cellfun(@numel, rows));
data = NaN(n, nColumns);

for k = 1:n
    data(k, 1:numel(rows{k})) = rows{k};
end

% Se o monitor imprimiu o log várias vezes, usar o último bloco.
lastReset = find(diff(data(:, 1)) < 0, 1, 'last');

if ~isempty(lastReset)
    data = data(lastReset + 1:end, :);
end

fprintf('Log importado: %d amostras e %d colunas.\n', ...
    size(data, 1), size(data, 2));

%% 3. Extrair sinais

t = data(:, 1);
t = t - t(1);

state       = data(:, 2);
vRef        = data(:, 3);       % Referência ideal pedida pela missão
vEnc        = data(:, 17);      % Velocidade de avanço pelos encoders
xEnc        = data(:, 19);
yEnc        = data(:, 20);
headingEnc  = data(:, 21);      % [rad]
vRefLeft    = data(:, 26);      % Referência da roda esquerda [m/s]
vRefRight   = data(:, 27);      % Referência da roda direita [m/s]
headingGyro = data(:, 29);      % Heading já integrado no C++ [rad]

if numel(t) < 2 || any(diff(t) <= 0)
    error('Os tempos do ensaio têm de ser estritamente crescentes.');
end

if any(state == 41)
    % Full-path mission
    if ~any(state == 43) || ~any(state == 45)
        error('O log não contém todas as fases da trajetória completa.');
    end

    active = ismember(state, [40 41 42 43 44 45 47]);

else
    % Standalone straight mission
    if ~any(state == 10) || ~any(state == 11)
        error('O log não contém aceleração e travagem da missão em reta.');
    end

    active = ismember(state, [10 11 12]);
end

%% 4. Reconstruir a referência de avanço após o limitador

% Mixer do C++:
%   vRefLeft  = vLimited - vTurnLimited
%   vRefRight = vLimited + vTurnLimited
vLimited = 0.5 * (vRefLeft + vRefRight);

% Diferenças apenas entre duas amostras consecutivas com controlo ativo.
% Exclui a transição para o estado 2.
dt = diff(t);
validPair = active(1:end-1) & active(2:end);

if ~any(validPair)
    error('Amostras insuficientes para calcular a aceleração.');
end

tMid = 0.5 * (t(1:end-1) + t(2:end));
aRef = diff(vLimited) ./ dt;

tAcc = tMid(validPair);
aRef = aRef(validPair);

peakAccel = max([0; aRef]);     % Pico positivo [m/s^2]
peakBrake = min([0; aRef]);     % Pico negativo [m/s^2]
peakAbs   = max(abs(aRef));     % Maior módulo [m/s^2]

%% 5. Reconstruir as três trajetórias

% 1) Ideal: integrar a referência registada, com heading desejado de 0 rad.
xIdeal = cumtrapz(t, vRef);
yIdeal = zeros(size(t));

% 2) Heading do gyro + velocidade de avanço dos encoders.
xGyro = cumtrapz(t, vEnc .* cos(headingGyro));
yGyro = cumtrapz(t, vEnc .* sin(headingGyro));

% 3) Odometria dos encoders, já calculada no C++.
xEnc = xEnc - xEnc(1);
yEnc = yEnc - yEnc(1);

% Erro de posição em relação à posição ideal no mesmo instante.
errGyro = hypot(xGyro - xIdeal, yGyro - yIdeal);
errEnc  = hypot(xEnc  - xIdeal, yEnc  - yIdeal);

%% 6. Prints

fprintf('\n========== BASEBOT: ENSAIO EM RETA ==========\n');
fprintf('Log: %s\n', file);
fprintf('Amostras: %d | Duração registada: %.3f s\n', numel(t), t(end));

fprintf('\n--- ACELERAÇÃO DA REFERÊNCIA DE AVANÇO ---\n');
fprintf('mu usado no C++ (introduzido manualmente): %.2f\n', mu);
fprintf('Limite configurado, mu*g:                 %.4f m/s^2\n', accMax);
fprintf('Pico de aceleração da referência:          %+.4f m/s^2\n', peakAccel);
fprintf('Pico de travagem da referência:            %+.4f m/s^2\n', peakBrake);
fprintf('Máximo absoluto observado:                  %.4f m/s^2\n', peakAbs);
fprintf('Pico observado / limite configurado:        %.1f %%\n', ...
    100 * peakAbs / accMax);

fprintf('\n--- VELOCIDADE MÁXIMA ---\n');
fprintf('Referência ideal da missão:       %.4f m/s\n', max(vRef));
fprintf('Referência após o limitador:      %.4f m/s\n', max(vLimited(active)));
fprintf('Velocidade estimada por encoders: %.4f m/s\n', max(vEnc));

fprintf('\n--- TRAJETÓRIA: POSIÇÃO FINAL ---\n');
fprintf('%-27s %10s %10s %14s\n', ...
    'Trajetória', 'X [m]', 'Y [m]', 'Erro XY [m]');
fprintf('%-27s %10.4f %10.4f %14.4f\n', ...
    'Ideal', xIdeal(end), yIdeal(end), 0);
fprintf('%-27s %10.4f %10.4f %14.4f\n', ...
    'Gyro + velocidade encoder', xGyro(end), yGyro(end), errGyro(end));
fprintf('%-27s %10.4f %10.4f %14.4f\n', ...
    'Odometria dos encoders', xEnc(end), yEnc(end), errEnc(end));

fprintf('\n--- DESVIO DURANTE O PERCURSO ---\n');
fprintf('Máximo |Y|, gyro + velocidade encoder: %.2f cm\n', ...
    100 * max(abs(yGyro)));
fprintf('Máximo |Y|, odometria dos encoders:    %.2f cm\n', ...
    100 * max(abs(yEnc)));
fprintf('Heading final, gyro:                   %+.2f deg\n', ...
    rad2deg(headingGyro(end)));
fprintf('Heading final, encoders:               %+.2f deg\n', ...
    rad2deg(headingEnc(end)));
fprintf('=============================================\n');

%% 7. Figura 1: velocidade e aceleração

figure('Color', 'w', 'Name', 'Velocidade e aceleração');

subplot(2,1,1);
plot(t, vRef, 'k--', 'LineWidth', 1.6);
hold on;
plot(t(active), vLimited(active), 'b', 'LineWidth', 1.5);
plot(t, vEnc, 'r', 'LineWidth', 1.1);
grid on;
xlabel('Tempo [s]');
ylabel('Velocidade [m/s]');
title(sprintf('Ensaio em reta | miu = %.3f', mu));
legend('Referência ideal', 'Referência limitada', ...
       'Velocidade pelos encoders', 'Location', 'best');

subplot(2,1,2);
plot(tAcc, aRef, 'b', 'LineWidth', 1.3);
hold on;
yline( accMax, 'r--', '+\mu g');
yline(-accMax, 'r--', '-\mu g');
yline(0, 'k:');
grid on;
xlabel('Tempo [s]');
ylabel('Aceleração [m/s²]');
title('Aceleração da referência de avanço após o limitador');

%% 8. Figura 2: trajetória ideal vs gyro vs encoders

figure('Color', 'w', 'Name', 'Comparação das trajetórias');

subplot(2,1,1);
plot(xIdeal, yIdeal, 'k--', 'LineWidth', 2);
hold on;
plot(xGyro, yGyro, 'b', 'LineWidth', 1.5);
plot(xEnc, yEnc, 'r', 'LineWidth', 1.5);

plot(xIdeal(end), yIdeal(end), 'ks', 'HandleVisibility', 'off');
plot(xGyro(end), yGyro(end), 'bo', 'HandleVisibility', 'off');
plot(xEnc(end), yEnc(end), 'rx', 'HandleVisibility', 'off');

axis equal;
grid on;
xlabel('X [m]');
ylabel('Y [m]');
title('Trajetória no chão');
legend('Ideal', 'Heading do gyro + velocidade dos encoders', ...
       'Odometria dos encoders', 'Location', 'best');

% Mostra desvios de poucos centímetros, difíceis de ver em axis equal.
subplot(2,1,2);
plot(t, 100*yGyro, 'b', 'LineWidth', 1.5);
hold on;
plot(t, 100*yEnc, 'r', 'LineWidth', 1.5);
yline(0, 'k--');
grid on;
xlabel('Tempo [s]');
ylabel('Desvio lateral Y [cm]');
title('Desvio lateral estimado em relação à reta ideal');
legend('Gyro + velocidade dos encoders', 'Odometria dos encoders', ...
       'Ideal', 'Location', 'best');

%travagem menor: device-monitor-261111-113354
%travagem maior: device-monitor-261111-114312 APPROACH ESCOLHIDA

% Sem turnrate controller
%1.0m em casa (correção de heading nula): device-monitor-261111-160025 - desvio maximo 0.45cm
%1.5m em casa (correção de heading nula): device-monitor-261111-161530 - desvio maximo 3.08cm

% Com turnrate controller
%1.0m em casa (correção de heading nula): device-monitor-261111-162638 - desvio maximo 1.08cm
%1.5m em casa (correção de heading nula): device-monitor-261111-163921 - desvio maximo 1.50 cm

% (Nota: A velocidade mais alta nos 1.5m está a deixar a performance muito imprevisível, 
% temos de perceber se isso também acontece na sala, se acontecer não vale a pena isto para poupar 0.1s, 
% mantemos a approach anterior de apenas 1.5 m/s de velocidade máxima com travagem menor)


% Trajetória Completa

% Sem turnrate controller
% para já nada de jeito

% Com turnrate controller
% para já nada de jeito

