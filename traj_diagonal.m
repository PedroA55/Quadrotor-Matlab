function [X_d] = traj_diagonal(t, state_d)
% Esta função calcula a trajetória de subida vertical
% parâmetros: Tfinal - Tempo máximo da simulação
% parâmetros: tStepMax - Passo de tempo da simulação

for i=1:length(t)
    state_d(i,1)=0.5*t(i);      %Posição em X
    state_d(i,2)=0.5*t(i);      %Posição em Y
    state_d(i,3)=t(i);          %Posição em Z 
    state_d(i,4)=0.5;           %Velocidade em x
    state_d(i,5)=0.5;           %Velocidade em y
    state_d(i,6)=1;             %Velocidade em z
    state_d(i,7)=0;             %Ângulo de rolagem
    state_d(i,8)=0;             %Ângulo de arfagem
    state_d(i,9)=0;             %Ângulo de guinada
    state_d(i,10)=0;            %Velocidade angular em roll
    state_d(i,11)=0;
    state_d(i,12)=0;
end
X_d=state_d;
end