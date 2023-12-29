function [M2, M3, M4] = controller_SDRE(x,wd,z, Ao, Bo, Qo, Ro, C, params)
% Controlador de atitude. 
% Controle n�o linear SDRE - State Dependent Ricatti Equation
% Entradas: 
% x(t): Estados
% wd: Diferen�a nas velocidades angulares
% z(t): Trajet�ria de refer�ncia. Nessa situa��o, yaw vem diretamente da
% ref enquanto roll e pitch vem do controle de posi��o.
% Ao: Matriz sub�tima dos estados
% Bo: Matriz sub�tima da a��o de controle 
% params: Vetor de par�metros do modelo

% Passo 1 - Atualizar as matrizes sub�timas
Ao(4,5) = -(params(6)/params(3))*wd;  %-(Jr/Ixx)*wd
Ao(4,6) = x(11)*params(8);            % dtheta*I1
Ao(5,4) = (params(6)/params(4))*wd;   %(Jr/Iyy)*wd
Ao(5,6) = x(10)*params(9);            % dphi*I2
Ao(6,5) = x(11)*params(10);           % dtheta*I3
% Passo 2 - Com as matrizes Q e R redefinir as matrizes E(x), V(x) e W(x)
E = Bo*inv(Ro)*Bo'; V = C'*Qo*C; W = C'*Qo;
% Passo 3 - Resolver a Eq. Alg�brica de Ricatti
P = are(Ao, E, V);
% Passo 4 - Ganhos K(x) e Kz(x)
K = inv(Ro)*Bo'*P;
Kz =inv(Ro)*Bo'*inv(P*E-Ao')*W;
% Passo 5 - Sinal de controle Final
u = -K*x(7:12)' + Kz*(z'); M2 = u(1); M3 = u(2); M4 = u(3);
end