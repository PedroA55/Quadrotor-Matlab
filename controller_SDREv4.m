function [M2, M3, M4] = controller_SDREv4(x,wd,z, Qo, Ro, C, params,alpha)
% Controlador de atitude na versão 4 - Full nonlinear trabalho Pedro 2024. 
% Controle não linear SDRE - State Dependent Ricatti Equation
% Entradas: 
%   x(t): Estados (p, q, r)
%   wd: Diferença nas velocidades angulares
%   z(t): Trajetória de referência. Nessa situação, yaw vem diretamente da
%   ref enquanto roll e pitch vem do controle de posição
% params: Vetor de parâmetros do modelo
% Ts: Tempo de amostragem
%=========================================================================%
Jr = params(6); L = params(7);
Ixx = params(3); Iyy = params(4); Izz = params(5);
I1 = params(8);  I2 = params(9);  I3 = params(10);

% Passo 0: Obter as derivadas dos ângulos de Euler
dang = z(4:6)';
% Fazer uma filtragem nessas derivadas (Filtro de primeira ordem)

% Obtendo pqr_des
T = [1 0 -sin(z(2));0 cos(z(1)) sin(z(1))*cos(z(2)); 0 -sin(z(1)) cos(z(1))*cos(z(2))];
pqr_des = T*dang;

% Passo 1: Atualizar a matriz Ao
% Estrutura escolhida: alpha = [alpha1 alpha2 alpha3]
Ao = zeros(3);
Ao(1,2) = alpha(1)*I1*x(12)-(Jr*wd)/Ixx;
Ao(1,3) = (1-alpha(1))*I1*x(11);
Ao(2,1) = alpha(2)*I2*x(12)-(Jr*wd)/Iyy;
Ao(2,3) = (1-alpha(2))*I2*x(10);
Ao(3,1) = alpha(3)*I3*x(11);
Ao(3,2) = (1-alpha(3))*I3*x(10);

Bo = zeros(3);
Bo(1,1)=L/Ixx;
Bo(2,2)=L/Iyy;
Bo(3,3)=L/Izz;

% Passo 2 - Com as matrizes Q e R redefinir as matrizes E(x), V(x) e W(x)
E = Bo*inv(Ro)*Bo'; V = C'*Qo*C; W = C'*Qo;
% Passo 3 - Resolver a Eq. Algébrica de Ricatti
P = are(Ao, E, V);
% Passo 4 - Ganhos K(x) e Kz(x)
K = inv(Ro)*Bo'*P;
Kz =inv(Ro)*Bo'*inv(P*E-Ao')*W;
% Passo 5 - Sinal de controle Final
u = -K*x(10:12)' + Kz*(z(4:6)'); M2 = u(1); M3 =  u(2); M4 = u(3);
end

