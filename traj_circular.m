function [X_d] = traj_circular(t, state_d)
% Esta fun��o calcula a trajet�ria circular
% par�metros: Tfinal - Tempo m�ximo da simula��o
% par�metros: tStepMax - Passo de tempo da simula��o
w=pi/10; %frequ�ncia Angular
for i=1:length(t)
    state_d(i,1)=cos(w*t(i));    %x
    state_d(i,2)=sin(w*t(i));    %y
    state_d(i,3)=10;             %z
    state_d(i,4)=-sin(w*t(i))*w; %dx
    state_d(i,5)= cos(w*t(i))*w; %dy
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