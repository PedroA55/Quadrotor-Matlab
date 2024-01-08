function [M2, M3, M4] = controller_SDREv3(x,wd,z, Qo, Ro, C, params)
% Controlador de atitude na versão 3 - Full nonlinear trabalho Allan Carlos 2018. 
% Controle não linear SDRE - State Dependent Ricatti Equation
% Entradas: 
% x(t): Estados
% wd: Diferença nas velocidades angulares
% z(t): Trajetória de referência. Nessa situação, yaw vem diretamente da
% ref enquanto roll e pitch vem do controle de posição.
% Ao: Matriz subótima dos estados
% Bo: Matriz subótima da ação de controle 
% params: Vetor de parâmetros do modelo
Jr = params(6); L = params(7);
Ixx = params(3); Iyy = params(4); Izz = params(5);
I1 = params(8);  I2 = params(9);  I3 = params(10);

% Passo 1 - Atualizando os termos das matrizes subótimas
Sef = sin(x(7)); Cf = cos(x(7)); %Tf = tan(x(7)); 
St = sin(x(8)); Ct = cos(x(8)); Tt = tan(x(8)); 
%coeficientes
a1 = Tt*(1-(Sef^2)*I2 + (Cf^2)*I3);
a2 = St*Sef*Cf*(I2+I3);
a3 = 1/Ct + Ct*(Cf^2 - Sef^2)*I1 + St*(Sef^2)*Tt*I2 - St*(Cf^2)*Tt*I3;
a4 = -Sef*Cf*I1; 
a5 = Sef*Cf*((Ct^2)*I1-(St^2)*(I2+I3)); % Aqui pode ter um erro
a6 = -Sef*Cf*(I3+I2); a7 = Ct*(-1 + (Cf^2)*I2 - (Sef^2)*I3);
a8 = St*Sef*Cf*(I2+I3);
a9 = -St*Ct*((Cf^2)*I2-(Sef^2)*I3);
a10 = (1/Ct)*(1 -(Sef^2)*I2 + (Cf^2)*I3);
a11 = Sef*Cf*(I2+I3); a12 = Tt*(1+(Sef^2)*I2-(Cf^2)*I3);
a13 = -St*Sef*Cf*(I2+I3);
b1 = L/Ixx; b2 = (L*Tt*Sef)/Iyy; b3 = (L*Tt*Cf)/Izz;
b4 = (L*Cf)/Iyy; b5 = -(L*Sef)/Izz; b6 = (L*Sef)/(Ct*Iyy);
b7 = (L*Cf)/(Ct*Izz);
c1 = -(Cf*Jr*wd)/Ixx; c2 = -((Ct*Sef/Ixx)+(Tt*St*Sef/Iyy))*Jr*wd;
c3 = (Tt*Sef*Jr*wd)/Iyy;
c4 = -(St*Cf*Jr*wd)/Iyy;
c5 = (Cf*Jr*wd)/Iyy;
c6 = -(St*Sef*Jr*wd)/(Ct*Iyy);
c7 = (Sef*Jr*wd)/(Ct*Iyy);
A88 = a1*x(11)+ a2*x(12) +c3;
A810 = a4*x(11) + c1;
A812 = a3*x(11) + a5*x(12) + c2;
A108 = a6*x(11)+ a7*x(12) +c5;
A1010 = a8*x(12);
A1012 = a9*x(12) + c4;
A128 = a10*x(11)+ a11*x(12) +c7;
A1210 = a12*x(12);
A1212 = a13*x(12) + c6;
% Matriz Ao
Ao = zeros(6);
Ao(1,4)= 1; Ao(2,5)= 1; Ao(3,6)= 1;
Ao(4,4)= A88; Ao(4,5)= A810; Ao(4,6)= A812;
Ao(5,4)= A108; Ao(5,5)= A1010; Ao(5,6)= A1012;
Ao(6,4)= A128; Ao(6,5)= A1210; Ao(6,6)= A1212;
% Matriz Bo
Bo = zeros(6,3);
Bo(4,1)=b1; Bo(4,2)= b2; Bo(4,3)=b3;
Bo(5,2)=b4; Bo(5,3)= b5;
Bo(6,2)=b6; Bo(6,3)= b7;

Co = ctrb(Ao,Bo); % Matriz de Controlabilidade
rank(Co);
% Passo 2 - Com as matrizes Q e R redefinir as matrizes E(x), V(x) e W(x)
E = Bo*inv(Ro)*Bo'; V = C'*Qo*C; W = C'*Qo;
% Passo 3 - Resolver a Eq. Algébrica de Ricatti
P = are(Ao, E, V);
% Passo 4 - Ganhos K(x) e Kz(x)
K = inv(Ro)*Bo'*P;
Kz =inv(Ro)*Bo'*inv(P*E-Ao')*W;
% Passo 5 - Sinal de controle Final
u = -K*x(7:12)' + Kz*(z'); M2 = u(1); M3 = u(2); M4 = u(3);
end