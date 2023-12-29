function [X_d] = traj_diagonal(t, state_d)
% Esta fun��o calcula a trajet�ria de subida vertical
% par�metros: Tfinal - Tempo m�ximo da simula��o
% par�metros: tStepMax - Passo de tempo da simula��o

for i=1:length(t)
    state_d(i,1)=0.5*t(i);      %Posi��o em X
    state_d(i,2)=0.5*t(i);      %Posi��o em Y
    state_d(i,3)=t(i);          %Posi��o em Z 
    state_d(i,4)=0.5;           %Velocidade em x
    state_d(i,5)=0.5;           %Velocidade em y
    state_d(i,6)=1;             %Velocidade em z
    state_d(i,7)=0;             %�ngulo de rolagem
    state_d(i,8)=0;             %�ngulo de arfagem
    state_d(i,9)=0;             %�ngulo de guinada
    state_d(i,10)=0;            %Velocidade angular em roll
    state_d(i,11)=0;
    state_d(i,12)=0;
end
X_d=state_d;
end