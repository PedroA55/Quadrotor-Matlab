function J_Total = cost_function(Ref_att, Real_att, cmd, Qo, Ro,t)
% Funcao para calcular o indice de desempenho
% Entradas:
%   Ref_att: Referência de atitude em taxa pqr_des
%   Real_att: Real atitude em taxa excutada pelo modelo (ou pela aeronave real)
%   cmd: Comando enviado pelo controlador de taxa
%   Qo,Ro: Matrizes de ponderação no SDRE

err = (Ref_att - Real_att).*(pi/180); % Taxa em rad/s
u   = cmd;
J   = zeros(length(t),1);
Ju  = zeros(length(t),1);
Je  = zeros(length(t),1);
for i=1:length(t)
    Ju(i) = 0.5*u(i,:)*Ro*u(i,:)'; % cuidado com esse transposto aqui. Nesse código ele está trocado por causa da forma como vem o vetor de fora
    Je(i) = 0.5*err(i,:)*Qo*err(i,:)';
    J(i)  = Je(i) + Ju(i);
end
J_Total = [J, Je, Ju];
end