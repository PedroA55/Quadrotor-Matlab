function [X_d] = traj_vertical(t, state_d)
% Esta função calcula a trajetória de subida vertical
% parâmetros: Tfinal - Tempo máximo da simulação
% parâmetros: tStepMax - Passo de tempo da simulação

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