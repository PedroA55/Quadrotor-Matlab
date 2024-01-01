% ============ Mestrado Engenharia Mec�nica - UFABC ===================== %
% Nome: Pedro Henrique Anjos da Costa         RA:21202310396
% Projeto 1: Controladores PID, LQR e SDRE para VANT Quadrotor
% Orientador: Diego Paolo Ferruzzo Correa
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear;
close all;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Par�metros do modelo de simula��o (Modelo GYRO-200ED-X4)
% Esse modelo � o mesmo implementado no ROS/Gazebo
g = 9.81;           % acelera��o da gravidade
m = 1.03;           % massa total do ve�culo
Ixx = 16.83e-3;     % Momento de in�rcia em x
Iyy = 16.836e-3;    % Momento de in�rcia em y
Izz = 28.34e-3;     % Momento de in�rcia em z
Jr = 5e-5;          % Momento de in�rcia do rotor (assume-se que seja igual para todos os motores)
L = 0.26;           % Dist�ncia do rotor at� o CG
% constantes das equa��es
I1 = (Iyy-Izz)/Ixx; I2 = (Izz-Ixx)/Iyy; I3 = (Ixx-Iyy)/Izz;
params = [g m Ixx Iyy Izz Jr L I1 I2 I3];
%% Modelo dos motores 
% Kp = 0.9182;    % ganho (Precisa ainda ser ajustado)
% tau_a = 0.0569; % polo (Precisa ainda ser ajustado)
% tau_s = 0.0452; % atraso de transporte (Precisa ainda ser ajustado)
% sem limita��o dos motores por enquanto
%rpm_max = 15400;
%rpm_min = 8700;
% inlcuir din�mica dos motores?
% -1 N�O
% 1  SIM
incluir_din_motor = -1;
%% Configura��o de simula��o
Tfinal=20;
tStepMax=1e-3; Ts = tStepMax; % Tempo de amostragem
t=0:tStepMax:Tfinal; %Define o vetor de tempo
t=t';
state_d = zeros(length(t),12); % Define o tamanho da matriz do vetor de estado
%% Trajet�ria escolhida
% ------------------------ Trajet�rias lineares ------------------------- %
%X_d = traj_hover(t, state_d);
%X_d=traj_vertical(t,state_d);
%X_d = traj_decola_hover(t, state_d);
%X_d=traj_diagonal(t,state_d);
%X_d = traj_estab_Atitude(t, state_d);
%X_d = traj_circular(t, state_d);
%X_d=traj_senoide(t,state_d);
%X_d=traj_helipcoidal(t,state_d);
%X_d=traj_oito(t,state_d);
% ------------------------ Trajet�rias complexas ------------------------ %
uso_waypoint = 1; % decido se a trajet�ria � com way_point
%X_d = traj_waypoint(t, Ts);
X_d = traj_waypoint2(t, Ts);
if uso_waypoint % corre��o do tamanho do vetor tempo.
    m = size(X_d,1);
    t = linspace(0,Tfinal,m)';
end

%% Condi��es iniciais dependendo da minha trajet�ria X_d
% Condi��es iniciais para as posi��es
x0 = X_d(1,1);
y0 = X_d(1,2);
z0 = X_d(1,3);
% Condi��es iniciais para as velocidades translacionais
vx0 = X_d(1,4);
vy0 = X_d(1,5);
vz0 = X_d(1,6);
% Condi��es iniciais �ngulos de Euler
phi0 = X_d(1,7);
theta0 = X_d(1,8);
psi0 = X_d(1,9);
% Condi��es iniciais de p,q,r
p0 = X_d(1,10);
q0 = X_d(1,11);
r0 = X_d(1,12);
%Condi��es Iniciais
xCI =[x0 y0 z0 vx0 vy0 vz0 phi0 theta0 psi0 p0 q0 r0];
x = [x0 y0 z0 vx0 vy0 vz0 phi0 theta0 psi0 p0 q0 r0];
x_ant = x; % vari�vel auxiliar para guardar estados anteriores
wd = 0; % Diferen�a entre as rota��es 
U = [0 0 0 0]';
acc = [0 0 0]; % Acelera��o de posi��o inicial
I = [0 0 0]; % Termo integrativo do controle de posi��o incio
Iz = 0;
erro_ant = 0;
%% Controlador �timo linear, LQR, para din�mica rotacional
% Espa�o de estados: Xdot(t)=A*X(t)+B*u(t)
% Lembrando que meu vetor de estado e as entradas neste controlador s�o: 
% X=[phi;theta;psi;dot_phi;dot_theta;dot_psi]
% u=[Torque_roll(u2);Torque_pitch(u3);Torque_yaw(u4)]
A=zeros(6);
A(1,4) = 1;
A(2,5) = 1;
A(3,6) = 1;
B=zeros(6,3);
B(4,1)=L/Ixx;
B(5,2)=L/Iyy;
B(6,3)=L/Izz;
C = eye(6,6);
%C = [zeros(3) eye(3)]; % a sa�da s�o os �ngulos de Euler (phi, theta, psi)
Co = ctrb(A,B); % Matriz de Controlabilidade
rank(Co);
fprintf('\n O posto da Matriz de Controlabilidade      = %g',rank(Co))
fprintf('\n Dimens�o da Matriz A                       = %g \n',6)
% Defini��o das matrizes de podera��o Q e R
q11 = 10^(3); % pondera mais os �ngulos
q22 = 10^(3);
q33 = 10^(3);
q44 = 1; % Restringe menos a taxa (p,q,r)
q55 = 1;
q66 = 1;
Q = diag([q11,q22,q33,q44,q55,q66]);
r11 = 0.1;
r22 = 0.1;
r33 = 0.1;
R = [r11 0 0;0 r22 0; 0 0 r33];
% Resolu��o da eq. Ricatti pelo MATLAB
P = are(A, B*inv(R)*B',C'*Q*C);
% A solu��o alg�brica da eq alg�brica adicional (no caso de tracking)
G = inv(P*B*inv(R)*B'-A')*C'*Q;
% Ganhos para a lei de controle
K = inv(R)*B'*P;
Kz = inv(R)*B'*G;
%% Controlador �timo n�o-linear, SDRE, para a din�mica rotacional
% Existem diversas configura��es de matrizes A e B para casos sub�timos.
% Estou escolhendo de acordo com o trabalho de Voos 2006, mas um pouco
% diferente
Ao = zeros(6); % Matriz dos estados sub-�tima
Ao(1,4) = 1;
Ao(2,5) = 1;
Ao(3,6) = 1;
Ao(4,5) = -(Jr/Ixx)*wd;
Ao(4,6) = x(11)*I1;
Ao(5,4) = (Jr/Iyy)*wd;
Ao(5,6) = x(10)*I2;
Ao(6,5) = x(11)*I3;
Bo=zeros(6,3);
Bo(4,1)=L/Ixx;
Bo(5,2)=L/Iyy;
Bo(6,3)=L/Izz;
%Sa�da
C = eye(6,6);
% Testei a controlabilidade para x0 e este sistema � control�vel nesta
% condi��o.
% Defini��o das matrizes de podera��o Q e R
q11o = 10^(3); % pondera mais os �ngulos
q22o = 10^(3);
q33o = 10^(3);
q44o = 1; % Restringe menos a taxa (p,q,r)
q55o = 1;
q66o = 1;
Qo = diag([q11o,q22o,q33o,q44o,q55o,q66o]);
r11o = 10^(-1);
r22o = r11;
r33o = r11;
Ro = [r11o 0 0;0 r22o 0; 0 0 r33o];

%% Roda o modelo n�o linear
% OBS importante: Ao rodar apenas esse trecho v�rias vezes, toma cuidado
% pois o vetor x n�o est� sendo resetado e isso faz diferen�a na sua
% an�lise. portanto ao rodar esse c�digo de novo, por favor resetar os
% estados, os integradores, os sinais de controle e etc.
[x, I,acc, U, x_ant, wd] = ResetSimulation(xCI);
for i=2:(length(t)-1)
    %---------------------- Controle de posi��o --------------------------%
    % Preciso evitar o wind-up na parte integral: 
    if(X_d(i,1)-x(1))<= 0
        I(1) = 0;
    end
    if(X_d(i,2)-x(2))<= 0
        I(2) = 0;
    end
    if(X_d(i,3)-x(3))<= 0
        I(3) = 0;
    end
    %-------------------- acelera��o de posi��o --------------------------%
    ddx = (X_d(i,4) - X_d(i-1,4))/Ts; ddy = (X_d(i,5) - X_d(i-1,5))/Ts; ddz = (X_d(i,6) - X_d(i-1,6))/Ts; 
    acc = [ddx, ddy, ddz]; % ser� que � a melhor forma de derivar ?
    %PID para posi��o
    [u1, phi_des, theta_des, psi_des, I] = controller_pos(x, X_d(i,:), acc, params, I, Ts);
    
    %-------- Gerando a trajet�ria de refer�ncia para a atitude ----------%
    dphi = (phi_des - x_ant(7))/Ts; dtheta = (theta_des - x_ant(8))/Ts;
    dpsi = (psi_des - x_ant(9))/Ts;
    z = [phi_des, theta_des, psi_des, dphi, dtheta, dpsi];
    
    %------------------- Controladores de atitude ------------------------%
    [u2, u3, u4] = controller_att(phi_des, theta_des, psi_des, x, X_d(i,:), params);
    %[u2, u3, u4] = controller_LQT(x,z, K, Kz);
    %[u2, u3, u4] = controller_SDRE(x,wd,z, Ao, Bo, Qo, Ro, C, params);
    
    %-------------------- Sinal de Controle Final ------------------------%
    U = [u1, u2, u3, u4]';
    
    %------------------------ Planta n�o linear --------------------------%
    [x, xang, xpos, wd] = QuadModel(x,U,Ts);
    x_ant = x;
    
    %--------------------- Guardando os vetores --------------------------%
    Todos_estados(i+1,:) = x;
    Dados_posicao(i+1,:) = xpos;
    Dados_atitude(i+1,:) = xang;
    Ang_Target(i+1,:) = z;
    Sinal_controle(i+1,:) = U';
    Dif_wd(i+1,:) = wd;
end
posicao_x = Dados_posicao(:,1);
posicao_y = Dados_posicao(:,2);
posicao_z = Dados_posicao(:,3);
rolagem = Dados_atitude(:,1);
arfagem = Dados_atitude(:,2);
guinada = Dados_atitude(:,3);
Controle = Sinal_controle;
%% Gr�ficos
figure
subplot(3,2,1)
plot(t,posicao_x, 'b')
hold on
plot(t,X_d(:,1),'--r')
grid minor
xlabel('t (segundos)','Fontsize',11)
ylabel('posi��o em x (m)','Fontsize',11)
legend('x','x desejado')
hold on
subplot(3,2,3)
plot(t,posicao_y, 'b')
grid minor
hold on
plot(t,X_d(:,2),'--r')
xlabel('t (segundos)','Fontsize',11)
ylabel('posi��o em y (m)','Fontsize',11)
legend('y','y desejado')
subplot(3,2,5)
plot(t,posicao_z, 'b')
hold on
plot(t,X_d(:,3),'--r')
grid minor
xlabel('t (segundos)','Fontsize',11)
ylabel('altitude (m)','Fontsize',11)
legend('z','z desejado')
subplot(3,2,2)
plot(t,rolagem, 'b')
hold on
plot(t,X_d(:,7),'--r')
hold on
plot(t, Ang_Target(:,1), '--g')
grid minor
xlabel('t (segundos)','Fontsize',11)
ylabel('\phi (rad)','Fontsize',11)
legend('\phi','\phi desejado', '\phi alvo')
subplot(3,2,4)
plot(t,arfagem, 'b')
hold on
plot(t,X_d(:,8),'--r')
hold on
plot(t, Ang_Target(:,2), '--g')
grid minor
xlabel('t (segundos)','Fontsize',11)
ylabel('\theta (rad)','Fontsize',11)
legend('\theta','\theta desejado', '\theta alvo')
subplot(3,2,6)
plot(t,guinada, 'b')
hold on
plot(t,X_d(:,9),'--r')
hold on
plot(t, Ang_Target(:,3), '--g')
grid minor
xlabel('t (segundos)','Fontsize',11)
ylabel('\psi (rad)','Fontsize',11)
legend('\psi','\psi desejado', '\psi alvo')
suptitle('Resposta dos 6 graus de liberdade')
% Plotar a trajet�ria em 3D
figure 
plot3(posicao_x,posicao_y,posicao_z, 'b', 'LineWidth', 2)
hold on
plot3(X_d(:,1),X_d(:,2),X_d(:,3), '-.r', 'LineWidth', 2)
grid on
xlabel('x coordinates')
ylabel('y coordinates')
zlabel('z coordinates')
legend('Real Trajectory','Desired Trajectory')
title('Quadrotor trajectory')
figure
plot(t,X_d(:,1),'--',t,X_d(:,2),'--',t,X_d(:,3),'--',t,X_d(:,7),'--',t,X_d(:,8),'--',t,X_d(:,9),'--')
hold on
plot(t,posicao_x,t,posicao_y,t,posicao_z,t,rolagem,t,arfagem,t,guinada)
grid on
xlabel('t (segundos)','Fontsize',13)
ylabel('vari�veis de sa�da','Fontsize',13)
legend('x desejado','y desejado','z desejado','\phi desejado','\theta desejado','\psi desejado','x','y','z','\phi','\theta','\psi')
% Gr�fico para a trajet�ria em oito
% plano yz
figure
plot(posicao_y,posicao_z, 'r')
hold on 
plot(X_d(:,2),X_d(:,3), 'b')
grid minor
ylabel('Coordenadas em y')
zlabel('Coordenadas em z')
legend('Real Trajectory','Desired Trajectory')
title('Trajet�ria do Quadrotor no plano yz')
% Erros na posi��o
figure;
subplot(3,1,1)
plot(t,(X_d(:,3)-posicao_z), 'b')
grid minor
xlabel('Tempo (s)')
ylabel('Erro (m)')
title('Erro na altitude')
subplot(3,1,2)
plot(t,(X_d(:,1)-posicao_x), 'g')
grid minor
xlabel('Tempo (s)')
ylabel('Erro (m)')
title('Erro na posi��o x')
subplot(3,1,3)
plot(t,(X_d(:,2)-posicao_y), 'r')
grid minor
xlabel('Tempo (s)')
ylabel('Erro (m)')
title('Erro na posi��o y')
% Sinal de controle


% Gr�fico com Momento, Taxa (desejada vs real), �ngulo (desejado vs real)
figure;
subplot(3,1,1)
plot(t,Controle(:,2), 'b', 'LineWidth',1)
grid minor
ylabel('u_2 (Nm)')
title('Moment')
subplot(3,1,2)
plot(t,rolagem, 'b', 'LineWidth',1)
hold on
plot(t,Ang_Target(:,1), '-.r', 'LineWidth',1)
ylabel('\phi (rad)')
title ('roll')
legend('Simulated', 'Desired')
grid minor
subplot(3,1,3)
plot(t,Todos_estados(:,10), 'b', 'LineWidth',1)
hold on
plot(t,Ang_Target(:,4), '-.r', 'LineWidth',1)
ylabel('p (rad/s)')
title('roll rate')
legend('Simulated', 'Desired')
grid minor
sgtitle('Results PID attitude controller')
%% Aramazenamento de dados
% Estou experimentando controladores diferentes e portanto � interessante
% fazermos comparativos diretos entre eles. 
% Salvando todas as informa��es
DATA = [t, Todos_estados, X_d, Ang_Target, Controle];
save("PID_att.mat","DATA")