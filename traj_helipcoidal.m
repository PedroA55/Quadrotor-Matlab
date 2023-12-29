function [X_d] = traj_helipcoidal(t, state_d)
% Esta função calcula a trajetória helipcoidal
% parâmetros: Tfinal - Tempo máximo da simulação
% parâmetros: tStepMax - Passo de tempo da simulação
w=pi/120; %frequência Angular
% Nesse caso eu vou dar como input as velocidades desejadas.
% Precisa derivar as equações de movimento
%     x = cos(w*t(i))-1;  %x
%     y = sin(w*t(i));    %y
%     z = 0.001*t(i);     %z
for i=1:length(t)
    state_d(i,1)=cos(w*t(i))-1;    %x
    state_d(i,2)=sin(w*t(i));      %y
    state_d(i,3)=0.001*t(i);       %z
    state_d(i,4)=-sin(w*t(i))*w;   %u
    state_d(i,5)=cos(w*t(i))*w;    %v
    state_d(i,6)=0.001;            %w
    state_d(i,7)=0;                %phi
    state_d(i,8)=0;                %theta
    state_d(i,9)=0;                %psi
    state_d(i,10)=0;               %p
    state_d(i,11)=0;               %q
    state_d(i,12)=0;               %r
end
X_d=state_d;
end