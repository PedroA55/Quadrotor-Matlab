function y = Filter_butter(x,fc,Ts)
%FILTER_BUTTER Summary of this function goes here
% Configuração
fs = 1/Ts; % Frequência de amostragem (Hz)
% fc  Frequência de corte (Hz)
[b, a] = butter(2, fc/(fs/2), 'low'); % Filtro Butterworth de 2ª ordem

% Sinal de entrada (exemplo: seno + ruído)
%t = 0:1/fs:1-1/fs;
%x = sin(2*pi*10*t) + 0.5*randn(size(t));

% Aplicação do filtro
y = filter(b, a, x);

% Plot
% figure;
% subplot(2, 1, 1);
% plot(t, x);
% title('Sinal Original');
% xlabel('Tempo (s)');
% ylabel('Amplitude');
% 
% subplot(2, 1, 2);
% plot(t, y);
% title('Sinal Filtrado (Passa-Baixa)');
% xlabel('Tempo (s)');
% ylabel('Amplitude');

end

