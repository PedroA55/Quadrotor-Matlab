function [X_d] = traj_estab_Atitude(t, state_d)
% Esta função calcula a estabilização da atitude
% parâmetros: Tfinal - Tempo máximo da simulação
% parâmetros: tStepMax - Passo de tempo da simulação
    state_d(1,1)=0;               %x
    state_d(1,2)=0;               %y
    state_d(1,3)=1;               %z
    state_d(1,4)=0;               %dx
    state_d(1,5)=0;               %dy
    state_d(1,6)=0;               %dz
    state_d(1,7)=0*(pi/180);     %phi
    state_d(1,8)=0*(pi/180);     %theta
    state_d(1,9)= 0*(pi/180);    %psi
    state_d(1,10)=0;              %dot{phi}
    state_d(1,11)=0;              %dot{theta}
    state_d(1,12)=0;              %dot{psi}
for i=2:length(t)
    state_d(i,1)=0;              %x
    state_d(i,2)=0;              %y
    state_d(i,3)=1;              %z
    state_d(i,4)=0;              %dx
    state_d(i,5)=0;              %dy
    state_d(i,6)=0;              %dz
    state_d(i,7)=0;              %phi
    state_d(i,8)=0;              %theta
    state_d(i,9)=0;              %psi
    state_d(i,10)=0;             %dot{phi}
    state_d(i,11)=0;             %dot{theta}
    state_d(i,12)=0;             %dot{psi}
end
X_d=state_d;
end