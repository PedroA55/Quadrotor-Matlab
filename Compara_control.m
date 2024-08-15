% Código para plotar para comparar os controladores
%load("Trajetoria Waypoint\PID_att.mat");
%load("Traj_Waypoint_3\PID_att.mat");
%load("PID_stb.mat");
%load("PID_attF.mat");
load("PID_stbENOC.mat");
Dados_PID = DATA;
%load("Trajetoria Waypoint\LQT_att.mat");
%load("Traj_Waypoint_3\LQT_att.mat");
%load("LQT_stb.mat");
%load("LQT_attF.mat");
load("LQT_stbENOC.mat");
Dados_LQT = DATA;
%load("Trajetoria Waypoint\SDRE_att.mat");
%load("Traj_Waypoint_3\SDRE_att.mat");
%load("SDRE_stb.mat");
%load("SDRE_attF.mat");
load("SDRE_stbENOC2.mat");
Dados_SDRE = DATA;
% Cada dado é um vetor de: 
% DATA = [t, Todos_estados, X_d, Ang_Target, Controle];
t = Dados_PID(:,1);
% Gráfico com Momento, Taxa (desejada vs real), ângulo (desejado vs real)
figure;
subplot(3,1,1)
plot(t,Dados_PID(:,32), 'b', 'LineWidth',1)
hold on
plot(t,Dados_LQT(:,32), 'r', 'LineWidth',1)
hold on
plot(t,Dados_SDRE(:,32), 'g', 'LineWidth',2)
grid minor
ylabel('u_2 (Nm)')
legend('PID','LQT','SDRE')
title('Control effort')
subplot(3,1,2)
plot(t,Dados_PID(:,8), 'b', 'LineWidth',1)
hold on
plot(t,Dados_LQT(:,8), 'r', 'LineWidth',1)
hold on
plot(t,Dados_SDRE(:,8), 'g', 'LineWidth',2)
hold on
plot(t,Dados_PID(:,26), '-.k', 'LineWidth',1)
ylabel('\phi (rad)')
title ('roll')
legend('PID','LQT','SDRE', 'Desired')
grid minor
subplot(3,1,3)
plot(t,Dados_PID(:,11), 'b', 'LineWidth',1)
hold on
plot(t,Dados_LQT(:,11), 'r', 'LineWidth',1)
hold on
plot(t,Dados_SDRE(:,11), 'g', 'LineWidth',2)
ylabel('p (rad/s)')
title('roll rate')
legend('PID','LQT','SDRE')
grid minor
sgtitle('Results of Roll controllers')
figure;
subplot(3,1,1)
plot(t,Dados_PID(:,33), 'b', 'LineWidth',1)
hold on
plot(t,Dados_LQT(:,33), 'r', 'LineWidth',1)
hold on
plot(t,Dados_SDRE(:,33), 'g', 'LineWidth',2)
grid minor
ylabel('u_3 (Nm)')
legend('PID','LQT','SDRE')
title('Moment')
subplot(3,1,2)
plot(t,Dados_PID(:,9), 'b', 'LineWidth',1)
hold on
plot(t,Dados_LQT(:,9), 'r', 'LineWidth',1)
hold on
plot(t,Dados_SDRE(:,9), 'g', 'LineWidth',2)
hold on
plot(t,Dados_PID(:,27), '-.k', 'LineWidth',1)
ylabel('\theta (rad)')
title ('pitch')
legend('PID','LQT','SDRE', 'Desired')
grid minor
subplot(3,1,3)
plot(t,Dados_PID(:,12), 'b', 'LineWidth',1)
hold on
plot(t,Dados_LQT(:,12), 'r', 'LineWidth',1)
hold on
plot(t,Dados_SDRE(:,12), 'g', 'LineWidth',2)
ylabel('q (rad/s)')
title('pitch rate')
legend('PD','LQT','SDRE')
grid minor
sgtitle('Results of Pitch controllers')

figure; 
plot3(Dados_PID(:,2),Dados_PID(:,3),Dados_PID(:,4),'b', 'LineWidth',1)
hold on
plot3(Dados_LQT(:,2),Dados_LQT(:,3),Dados_LQT(:,4),'r', 'LineWidth',1)
hold on
plot3(Dados_SDRE(:,2),Dados_SDRE(:,3),Dados_SDRE(:,4),'g', 'LineWidth',2)
hold on
plot3(Dados_PID(:,14),Dados_PID(:,15),Dados_PID(:,16),'-.k', 'LineWidth',2.5)
grid minor
title('Position')
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')
legend('PD','LQT','SDRE', 'Desired')

figure; 
subplot(3,1,1)
plot(t,rolagem, 'g', 'LineWidth',1)
grid minor
hold on
plot(t,Ang_Target(:,1), '-.k', 'LineWidth',1)
ylabel('\phi (rad)')
legend('Simulated', 'Desired')
title('Roll')
subplot(3,1,2)
plot(t,arfagem, 'g', 'LineWidth',1)
hold on
plot(t,Ang_Target(:,2), '-.k', 'LineWidth',1)
ylabel('\theta (rad)')
title ('Pitch')
grid minor
subplot(3,1,3)
plot(t,guinada, 'g', 'LineWidth',1)
hold on
plot(t,Ang_Target(:,3), '-.k', 'LineWidth',1)
ylabel('\psi (rad)')
xlabel('time (s)')
title('Yaw')
grid minor
sgtitle('Results SDRE attitude controller')
%% Comparativo apenas de LQR e SDRE
% Gráfico com Momento, Taxa (desejada vs real), ângulo (desejado vs real)
figure;
subplot(3,1,1)
plot(t,Dados_LQT(:,32), 'r', 'LineWidth',1)
hold on
plot(t,Dados_SDRE(:,32), 'g', 'LineWidth',1)
grid minor
ylabel('u_2 (Nm)')
legend('LQT','SDRE')
title('Moment')
subplot(3,1,2)
plot(t,Dados_LQT(:,8), 'r', 'LineWidth',1)
hold on
plot(t,Dados_SDRE(:,8), 'g', 'LineWidth',1)
hold on
plot(t,Dados_LQT(:,26), '-.k', 'LineWidth',1)
ylabel('\phi (rad)')
title ('roll')
legend('LQT','SDRE', 'Desired')
grid minor
subplot(3,1,3)
plot(t,Dados_LQT(:,11), 'r', 'LineWidth',1)
hold on
plot(t,Dados_SDRE(:,11), 'g', 'LineWidth',1)
ylabel('p (rad/s)')
title('roll rate')
legend('LQT','SDRE')
grid minor
sgtitle('Results of Roll controllers')
figure;
subplot(3,1,1)
plot(t,Dados_LQT(:,33), 'r', 'LineWidth',1)
hold on
plot(t,Dados_SDRE(:,33), 'g', 'LineWidth',1)
grid minor
ylabel('u_3 (Nm)')
legend('LQT','SDRE')
title('Moment')
subplot(3,1,2)
plot(t,Dados_LQT(:,9), 'r', 'LineWidth',1)
hold on
plot(t,Dados_SDRE(:,9), 'g', 'LineWidth',1)
hold on
plot(t,Dados_LQT(:,27), '-.k', 'LineWidth',1)
ylabel('\theta (rad)')
title ('pitch')
legend('LQT','SDRE', 'Desired')
grid minor
subplot(3,1,3)
plot(t,Dados_LQT(:,12), 'r', 'LineWidth',1)
hold on
plot(t,Dados_SDRE(:,12), 'g', 'LineWidth',1)
ylabel('q (rad/s)')
title('pitch rate')
legend('LQT','SDRE')
grid minor
sgtitle('Results of Pitch controllers')
%% Poder de estabilizar do SDRE
load("SDRE_stbAt.mat");
Dados_SDREAt = DATA;
t_SDRE = Dados_SDREAt(:,1);
figure;
subplot(3,1,1)
plot(t_SDRE,Dados_SDREAt(:,5), 'r', 'LineWidth',1)
hold on
plot(t_SDRE,Dados_SDREAt(:,6), 'g', 'LineWidth',1)
hold on
plot(t_SDRE,Dados_SDREAt(:,7), 'b', 'LineWidth',1)
grid minor
ylabel('Angle [rad]')
legend('\phi','\theta', '\psi')
title('Angles')
subplot(3,1,2)
plot(t_SDRE,Dados_SDREAt(:,11), 'r', 'LineWidth',1)
hold on
plot(t_SDRE,Dados_SDREAt(:,12), 'g', 'LineWidth',1)
hold on
plot(t_SDRE,Dados_SDREAt(:,13), 'b', 'LineWidth',1)
ylabel('Angle rate [rad/s]')
title ('Angular Velocity')
hl = legend('$\dot{\phi}$','$\dot{\theta}$', '$\dot{\psi}$');
set(hl, 'Interpreter', 'latex');
grid minor
subplot(3,1,3)
plot(t_SDRE,Dados_SDREAt(:,32), 'r', 'LineWidth',1)
hold on
plot(t_SDRE,Dados_SDREAt(:,33), 'g', 'LineWidth',1)
hold on
plot(t_SDRE,Dados_SDREAt(:,34), 'b', 'LineWidth',1)
ylabel('u [N]')
title('Control signal')
legend('u_2','u_3', 'u_4')
grid minor
sgtitle('SDRE control for stability')