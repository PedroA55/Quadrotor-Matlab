% ============ Mestrado Engenharia Mecânica - UFABC ===================== %
% Nome: Pedro Henrique Anjos da Costa         RA:21202310396
% Projeto 1: Controladores PID, LQR e SDRE para VANT Quadrotor
% Orientador: Diego Paolo Ferruzzo Correa
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear;
close all;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Parâmetros do modelo de simulação (Modelo GYRO-200ED-X4)
% Esse modelo é o mesmo implementado no ROS/Gazebo
g = 9.81;           % aceleração da gravidade
m = 1.03;           % massa total do veículo
Ixx = 16.83e-3;     % Momento de inércia em x
Iyy = 16.836e-3;    % Momento de inércia em y
Izz = 28.34e-3;     % Momento de inércia em z
Jr = 5e-5;          % Momento de inércia do rotor (assume-se que seja igual para todos os motores)
L = 0.26;           % Distância do rotor até o CG
% constantes das equações
I1 = (Iyy-Izz)/Ixx; I2 = (Izz-Ixx)/Iyy; I3 = (Ixx-Iyy)/Izz;
params = [g m Ixx Iyy Izz Jr L I1 I2 I3];
%% Modelo dos motores 
% Kp = 0.9182;    % ganho (Precisa ainda ser ajustado)
% tau_a = 0.0569; % polo (Precisa ainda ser ajustado)
% tau_s = 0.0452; % atraso de transporte (Precisa ainda ser ajustado)
% sem limitação dos motores por enquanto
%rpm_max = 15400;
%rpm_min = 8700;
% inlcuir dinâmica dos motores?
% -1 NÃO
% 1  SIM
incluir_din_motor = -1;
%% Configuração de simulação
Tfinal=20;
tStepMax=1e-3; Ts = tStepMax; % Tempo de amostragem
t=0:tStepMax:Tfinal; %Define o vetor de tempo
t=t';
state_d = zeros(length(t),12); % Define o tamanho da matriz do vetor de estado
%% Trajetória escolhida
% ------------------------ Trajetórias lineares ------------------------- %
%X_d = traj_hover(t, state_d);
%X_d=traj_vertical(t,state_d);
%X_d = traj_decola_hover(t, state_d);
%X_d=traj_diagonal(t,state_d);
%X_d = traj_estab_Atitude(t, state_d);
%X_d = traj_circular(t, state_d);
%X_d=traj_senoide(t,state_d);
%X_d=traj_helipcoidal(t,state_d);
%X_d=traj_oito(t,state_d);
% ------------------------ Trajetórias complexas ------------------------ %
uso_waypoint = 1; % decido se a trajetória é com way_point
%X_d = traj_waypoint(t, Ts);
X_d = traj_waypoint2(t, Ts);
if uso_waypoint % correção do tamanho do vetor tempo.
    m = size(X_d,1);
    t = linspace(0,Tfinal,m)';
end

%% Condições iniciais dependendo da minha trajetória X_d
% Condições iniciais para as posições
x0 = X_d(1,1);
y0 = X_d(1,2);
z0 = X_d(1,3);
% Condições iniciais para as velocidades translacionais
vx0 = X_d(1,4);
vy0 = X_d(1,5);
vz0 = X_d(1,6);
% Condições iniciais ângulos de Euler
phi0 = X_d(1,7);
theta0 = X_d(1,8);
psi0 = X_d(1,9);
% Condições iniciais de p,q,r
p0 = X_d(1,10);
q0 = X_d(1,11);
r0 = X_d(1,12);
%Condições Iniciais
xCI =[x0 y0 z0 vx0 vy0 vz0 phi0 theta0 psi0 p0 q0 r0];
x = [x0 y0 z0 vx0 vy0 vz0 phi0 theta0 psi0 p0 q0 r0];
x_ant = x; % variável auxiliar para guardar estados anteriores
wd = 0; % Diferença entre as rotações 
U = [0 0 0 0]';
acc = [0 0 0]; % Aceleração de posição inicial
I = [0 0 0]; % Termo integrativo do controle de posição incio
Iz = 0;
erro_ant = 0;
%% Controlador ótimo linear, LQR, para dinâmica rotacional
% Espaço de estados: Xdot(t)=A*X(t)+B*u(t)
% Lembrando que meu vetor de estado e as entradas neste controlador são: 
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
%C = [zeros(3) eye(3)]; % a saída são os ângulos de Euler (phi, theta, psi)
Co = ctrb(A,B); % Matriz de Controlabilidade
rank(Co);
fprintf('\n O posto da Matriz de Controlabilidade      = %g',rank(Co))
fprintf('\n Dimensão da Matriz A                       = %g \n',6)
% Definição das matrizes de poderação Q e R
q11 = 10^(3); % pondera mais os ângulos
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
% Resolução da eq. Ricatti pelo MATLAB
P = are(A, B*inv(R)*B',C'*Q*C);
% A solução algébrica da eq algébrica adicional (no caso de tracking)
G = inv(P*B*inv(R)*B'-A')*C'*Q;
% Ganhos para a lei de controle
K = inv(R)*B'*P;
Kz = inv(R)*B'*G;
%% Controlador ótimo não-linear, SDRE, para a dinâmica rotacional
% Existem diversas configurações de matrizes A e B para casos subótimos.
% Estou escolhendo de acordo com o trabalho de Voos 2006, mas um pouco
% diferente
Ao = zeros(6); % Matriz dos estados sub-ótima
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
%Saída
C = eye(6,6);
% Testei a controlabilidade para x0 e este sistema é controlável nesta
% condição.
% Definição das matrizes de poderação Q e R
q11o = 10^(3); % pondera mais os ângulos
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

%% Roda o modelo não linear
% OBS importante: Ao rodar apenas esse trecho várias vezes, toma cuidado
% pois o vetor x não está sendo resetado e isso faz diferença na sua
% análise. portanto ao rodar esse código de novo, por favor resetar os
% estados, os integradores, os sinais de controle e etc.
[x, I,acc, U, x_ant, wd] = ResetSimulation(xCI);
for i=2:(length(t)-1)
    %---------------------- Controle de posição --------------------------%
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
    %-------------------- aceleração de posição --------------------------%
    ddx = (X_d(i,4) - X_d(i-1,4))/Ts; ddy = (X_d(i,5) - X_d(i-1,5))/Ts; ddz = (X_d(i,6) - X_d(i-1,6))/Ts; 
    acc = [ddx, ddy, ddz]; % será que é a melhor forma de derivar ?
    %PID para posição
    [u1, phi_des, theta_des, psi_des, I] = controller_pos(x, X_d(i,:), acc, params, I, Ts);
    
    %-------- Gerando a trajetória de referência para a atitude ----------%
    dphi = (phi_des - x_ant(7))/Ts; dtheta = (theta_des - x_ant(8))/Ts;
    dpsi = (psi_des - x_ant(9))/Ts;
    z = [phi_des, theta_des, psi_des, dphi, dtheta, dpsi];
    
    %------------------- Controladores de atitude ------------------------%
    [u2, u3, u4] = controller_att(phi_des, theta_des, psi_des, x, X_d(i,:), params);
    %[u2, u3, u4] = controller_LQT(x,z, K, Kz);
    %[u2, u3, u4] = controller_SDRE(x,wd,z, Ao, Bo, Qo, Ro, C, params);
    
    %-------------------- Sinal de Controle Final ------------------------%
    U = [u1, u2, u3, u4]';
    
    %------------------------ Planta não linear --------------------------%
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
%% Gráficos
figure
subplot(3,2,1)
plot(t,posicao_x, 'b')
hold on
plot(t,X_d(:,1),'--r')
grid minor
xlabel('t (segundos)','Fontsize',11)
ylabel('posição em x (m)','Fontsize',11)
legend('x','x desejado')
hold on
subplot(3,2,3)
plot(t,posicao_y, 'b')
grid minor
hold on
plot(t,X_d(:,2),'--r')
xlabel('t (segundos)','Fontsize',11)
ylabel('posição em y (m)','Fontsize',11)
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
% Plotar a trajetória em 3D
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
ylabel('variáveis de saída','Fontsize',13)
legend('x desejado','y desejado','z desejado','\phi desejado','\theta desejado','\psi desejado','x','y','z','\phi','\theta','\psi')
% Gráfico para a trajetória em oito
% plano yz
figure
plot(posicao_y,posicao_z, 'r')
hold on 
plot(X_d(:,2),X_d(:,3), 'b')
grid minor
ylabel('Coordenadas em y')
zlabel('Coordenadas em z')
legend('Real Trajectory','Desired Trajectory')
title('Trajetória do Quadrotor no plano yz')
% Erros na posição
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
title('Erro na posição x')
subplot(3,1,3)
plot(t,(X_d(:,2)-posicao_y), 'r')
grid minor
xlabel('Tempo (s)')
ylabel('Erro (m)')
title('Erro na posição y')
% Sinal de controle


% Gráfico com Momento, Taxa (desejada vs real), ângulo (desejado vs real)
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
% Estou experimentando controladores diferentes e portanto é interessante
% fazermos comparativos diretos entre eles. 
% Salvando todas as informações
DATA = [t, Todos_estados, X_d, Ang_Target, Controle];
save("PID_att.mat","DATA")