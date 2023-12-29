function [X_d] = traj_oito(t, state_d)
% Esta fun��o calcula a trajet�ria em forma de oito
% par�metros: Tfinal - Tempo m�ximo da simula��o
% par�metros: tStepMax - Passo de tempo da simula��o
w=2*pi/120; %frequ�ncia Angular
for i=1:length(t)
    state_d(i,1)=0;
    state_d(i,2)=0.5*sin(0.5*t(i));
    state_d(i,3)=5*sin(t(i))+10;
    state_d(i,4)=0;
    state_d(i,5)=0.25*cos(0.5*t(i));
    state_d(i,6)=5*cos(t(i));
    state_d(i,7)=0;
    state_d(i,8)=0;
    state_d(i,9)=0;
    state_d(i,10)=0;
    state_d(i,11)=0;
    state_d(i,12)=0;
end
X_d=state_d;
end