% ============ Mestrado Engenharia Mec�nica - UFABC ===================== %
% Nome: Pedro Henrique Anjos da Costa         RA:21202310396
% Projeto 1: Controladores PID, LQR e SDRE para VANT Quadrotor
% Orientador: Diego Paolo Ferruzzo Correa
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear; clc;
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
w_min = 296; % Limite inferior para rota��o em rad/s
w_max = 657; % Limite superior para rota��o em rad/s

kf = 1.4351e-5;  % constante de empuxo
km = 2.4086e-7; % constante de torque
L = 0.26;       % dist�ncia ao CG 
 
Tmax = kf*((w_max)^2);
Tmin = kf*(w_min^2);
Moment_max = kf*L*abs((w_max^2)-(w_min^2));
MomentYaw_max = 2*km*abs((w_max^2)-(w_min^2));
fprintf('O empuxo m�ximo de cada motor �:  %4.2f N \n', Tmax)
fprintf('O empuxo m�nimo de cada motor �:  %4.2f N \n', Tmin)
fprintf('O limite superior de empuxo de controle �:  %4.2f N \n', Tmax*4)
fprintf('O limite inferior de empuxo de controle �:  %4.2f N \n', Tmin*4)
fprintf('O momento m�ximo de rolagem e arfagem �:  %4.2f N.m \n', Moment_max)
fprintf('O momento m�ximo de guinada �:  %4.2f N.m \n', MomentYaw_max)

% inlcuir din�mica dos motores?
% 0 N�O
% 1  SIM
incluir_din_motor = 0;
%% Configura��o de simula��o
Tfinal=10;
tStepMax=1e-3; Ts = tStepMax; % Tempo de amostragem
t=0:tStepMax:Tfinal; %Define o vetor de tempo
t=t';
state_d = zeros(length(t),12); % Define o tamanho da matriz do vetor de estado
%Limites dos motores
Vector_Tmax = zeros(length(t),1);
Vector_Tmin = zeros(length(t),1);
Vector_Mmax = zeros(length(t),1);
Vector_MYawmax = zeros(length(t),1);
for i=1:length(t)
    Vector_Tmax(i) = Tmax;
    Vector_Tmin(i) = Tmin;
    Vector_Mmax(i) = Moment_max;
    Vector_MYawmax(i) = MomentYaw_max;
end
%% Trajet�ria escolhida
% ------------------------ Trajet�rias lineares ------------------------- %
%X_d = traj_hover(t, state_d);
%X_d=traj_vertical(t,state_d);
%X_d = traj_decola_hover(t, state_d);
%X_d=traj_diagonal(t,state_d);
X_d = traj_estab_Atitude(t, state_d);
%X_d = traj_circular(t, state_d);
%X_d=traj_senoide(t,state_d);
%X_d=traj_helipcoidal(t,state_d);
%X_d=traj_oito(t,state_d);
%X_d = traj_step_angle(t, state_d);
% ------------------------ Trajet�rias complexas ------------------------ %
uso_waypoint = 0; % decido se a trajet�ria � com way_point
if uso_waypoint % corre��o do tamanho do vetor tempo.
    %X_d = traj_waypoint(t, Ts);
    X_d = traj_waypoint2(t, Ts);
    %X_d = traj_waypoint3(t, Ts);
    %X_d = traj_waypoint4(t, Ts);
    %X_d = traj_waypointFinal(t, Ts);
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
% phi0 = X_d(1,7);
% theta0 = X_d(1,8);
% psi0 = X_d(1,9);
phi0 = 30*(pi/180);
theta0 = 30*(pi/180);
psi0 = 45*(pi/180);
% Condi��es iniciais de p,q,r
p0 = X_d(1,10);
q0 = X_d(1,11);
r0 = X_d(1,12);
%Condi��es Iniciais
xCI =[x0 y0 z0 vx0 vy0 vz0 phi0 theta0 psi0 p0 q0 r0];
%x = [x0 y0 z0 vx0 vy0 vz0 phi0 theta0 psi0 p0 q0 r0];
x_ant = xCI; % vari�vel auxiliar para guardar estados anteriores
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
A(4,5) = -(Jr/Ixx)*wd;
A(5,4) = (Jr/Iyy)*wd;
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
q11 = 10^(2); % pondera mais os �ngulos
q22 = 10^(2);
q33 = 10^(2);
q44 = 10^(-2); % Restringe menos a taxa (p,q,r)
q55 = 10^(-2);
q66 = 10^(-2);
Q = diag([q11,q22,q33,q44,q55,q66]);
r11 = 10^(0);
r22 = 10^(0);
r33 = 10^(0);
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
%Sa�da
C = eye(6,6);
% Testei a controlabilidade para x0 e este sistema � control�vel nesta
% condi��o.
% Defini��o das matrizes de podera��o Q e R
q11o = 10^(2); % pondera os �ngulos
q22o = 10^(2);
q33o = 10^(2);
q44o = 10^(-2); % Restringe a taxa (p,q,r)
q55o = 10^(-2);
q66o = 10^(-2);
Qo = diag([q11o,q22o,q33o,q44o,q55o,q66o]);
r11o = 10^(0);
r22o = 10^(0);
r33o = 10^(0);
Ro = [r11o 0 0;0 r22o 0; 0 0 r33o];
% Defini��o das matrizes de podera��o Q e R
% q11o = 10^(-1); % pondera os �ngulos
% q22o = 10^(-1);
% q33o = 10^(-2);
% q44o = 10^(1); % Restringe a taxa (p,q,r)
% q55o = 10^(1);
% q66o = 10^(-1);
% Qo = diag([q11o,q22o,q33o,q44o,q55o,q66o]);
% r11o = 10^(0);
% r22o = 10^(0);
% r33o = 10^(0);
% Ro = [r11o 0 0;0 r22o 0; 0 0 r33o];

%% Roda o modelo n�o linear
% OBS importante: Ao rodar apenas esse trecho v�rias vezes, toma cuidado
% pois o vetor x n�o est� sendo resetado e isso faz diferen�a na sua
% an�lise. portanto ao rodar esse c�digo de novo, por favor resetar os
% estados, os integradores, os sinais de controle e etc.
[x, I,acc, U, x_ant, wd] = ResetSimulation(xCI);
tic
for i=2:(length(t)-2)
    %-------------------- acelera��o de posi��o --------------------------%
    ddx = (X_d(i,4) - X_d(i-1,4))/Ts; ddy = (X_d(i,5) - X_d(i-1,5))/Ts; ddz = (X_d(i,6) - X_d(i-1,6))/Ts; 
    acc(i,:) = [ddx, ddy, ddz]; % ser� que � a melhor forma de derivar ?
end
acc0 = [X_d(1,4), X_d(1,5), X_d(1,6)]; % primeiro elemento de acelera��o
acc = [acc0; acc]; %Agora tenho um vetor de mesmo tamanho mas com uma linha igual a zero
for i=1:(length(t)-1)
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
    %PID para posi��o
    [u1, phi_des, theta_des, psi_des, I] = controller_pos(x, X_d(i,:), acc(i,:), params, I, Ts);
    
    %-------- Gerando a trajet�ria de refer�ncia para a atitude ----------%
    dphi = (phi_des - x_ant(7))/Ts; dtheta = (theta_des - x_ant(8))/Ts;
    dpsi = (psi_des - x_ant(9))/Ts;
    z = [phi_des, theta_des, psi_des, dphi, dtheta, dpsi];
    
    %------------------- Controladores de atitude ------------------------%
    %[u2, u3, u4] = controller_att(phi_des, theta_des, psi_des, x, X_d(i,:), params);
    %[u2, u3, u4] = controller_LQT(x,z, K, Kz);
    %[u2, u3, u4] = controller_SDRE(x,wd,z, Ao, Bo, Qo, Ro, C, params);
    %[u2, u3, u4] = controller_SDREv2(x,wd,z, Ao, Bo, Qo, Ro, C, params);
    [u2, u3, u4] = controller_SDREv3(x,wd,z, Qo, Ro, C, params);
    %-------------------- Sinal de Controle Final ------------------------%
    U = [u1, u2, u3, u4]';
    %Obs: Ao repassar a a��o de controle diretamente para a planta, estou
    %simplificando o motor para apenas a um ganho unit�rio o que na realidade
    %n�o �. O motor tem limita��o de rota��o e apresenta uma din�mica de
    %primeira ordem que deve ser levada em considera��o em simula��es
    %futuras
    %------------------------ Modelo dos motores -------------------------%
    if incluir_din_motor
        [U, omega_real] = MotorDynamic(U);
        omega_motor(i+1,:) = omega_real';
    end
    %------------------------ Planta n�o linear --------------------------%
    %[x, xang, xpos, wd] = QuadModel(x,U,Ts);
    [x, xang, xpos, wd, Monit] = QuadModel2(x,U,Ts);
    x_ant = x;
    
    %--------------------- Guardando os vetores --------------------------%
    Todos_estados(i+1,:) = x;
    Dados_posicao(i+1,:) = xpos;
    Dados_atitude(i+1,:) = xang;
    Ang_Target(i+1,:) = z;
    Sinal_controle(i+1,:) = U';
    Dif_wd(i+1,:) = wd;
    %--------------------- Vetores para monitorar ------------------------%
    Monitorar(i+1,:) = Monit;
end
Todos_estados = [xCI; Todos_estados(2:end,:)];
Dados_posicao = [xCI(1:3); Dados_posicao(2:end,:)];
Dados_atitude = [xCI(7:9); Dados_atitude(2:end,:)];
Ang_Target = [X_d(1,7:12); Ang_Target(2:end,:)];

toc
dtAlg = toc*1000; % Tempo de compila��o 
posicao_x = Dados_posicao(:,1);
posicao_y = Dados_posicao(:,2);
posicao_z = Dados_posicao(:,3);
rolagem = Dados_atitude(:,1);
arfagem = Dados_atitude(:,2);
guinada = Dados_atitude(:,3);
Controle = [[4*Tmin, 0, 0, 0]; Sinal_controle(2:end,:)];
%% Gr�ficos
run graphics.m
%% Aramazenamento de dados
% Estou experimentando controladores diferentes e portanto � interessante
% fazermos comparativos diretos entre eles. 
% Salvando todas as informa��es
DATA = [t, Todos_estados, X_d, Ang_Target, Controle];
save("SDRE_stbENOC2.mat","DATA")