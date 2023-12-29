function [X_d] = traj_hover(t, state_d)
% Esta função calcula a trajetória pairado
% parâmetros: Tfinal - Tempo máximo da simulação
% parâmetros: tStepMax - Passo de tempo da simulação
for i=1:length(t)
    state_d(i,1)=0;               %x
    state_d(i,2)=0;               %y
    state_d(i,3)=1;               %z
    state_d(i,4)=0;               %u
    state_d(i,5)=0;               %v
    state_d(i,6)=0;               %w
    state_d(i,7)=0;              %phi
    state_d(i,8)=0;              %theta
    state_d(i,9)=0;              %psi
    state_d(i,10)=0;             %p
    state_d(i,11)=0;             %q
    state_d(i,12)=0;             %r
end
X_d=state_d;
end