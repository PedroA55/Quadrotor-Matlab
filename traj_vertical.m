function [X_d] = traj_vertical(t, state_d)
% Esta fun��o calcula a trajet�ria de subida vertical
% par�metros: Tfinal - Tempo m�ximo da simula��o
% par�metros: tStepMax - Passo de tempo da simula��o

for i=1:length(t)
    state_d(i,1)=0;
    state_d(i,2)=0;
    state_d(i,3)=2*t(i);
    state_d(i,4)=0;
    state_d(i,5)=0;
    state_d(i,6)=2;
    state_d(i,7)=0;
    state_d(i,8)=0;
    state_d(i,9)=0;
    state_d(i,10)=0;
    state_d(i,11)=0;
    state_d(i,12)=0;
end
X_d=state_d;
end