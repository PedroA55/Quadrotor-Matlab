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
%% Plotar a trajetória em 3D
figure 
plot3(posicao_x(1:end),posicao_y(1:end),posicao_z(1:end), 'g', 'LineWidth', 2)
hold on
plot3(X_d(1:end,1),X_d(1:end,2),X_d(1:end,3), '-.k', 'LineWidth', 2)
grid on
xlabel('x(m)')
ylabel('y(m)')
zlabel('z(m)')
legend('Real Trajectory', 'Desired trajectory')
title('Quadrotor trajectory')
% figure
% plot(t,X_d(:,1),'--',t,X_d(:,2),'--',t,X_d(:,3),'--',t,X_d(:,7),'--',t,X_d(:,8),'--',t,X_d(:,9),'--')
% hold on
% plot(t,posicao_x,t,posicao_y,t,posicao_z,t,rolagem,t,arfagem,t,guinada)
% grid on
% xlabel('t (segundos)','Fontsize',13)
% ylabel('variáveis de saída','Fontsize',13)
% legend('x desejado','y desejado','z desejado','\phi desejado','\theta desejado','\psi desejado','x','y','z','\phi','\theta','\psi')
% % Gráfico para a trajetória em oito
% plano yz
%figure
% plot(posicao_y,posicao_z, 'r')
% hold on 
% plot(X_d(:,2),X_d(:,3), 'b')
% grid minor
% ylabel('Coordenadas em y')
% zlabel('Coordenadas em z')
% legend('Real Trajectory','Desired Trajectory')
% title('Trajetória do Quadrotor no plano yz')
% %% Erros na posição
% figure;
% subplot(3,1,1)
% plot(t,(X_d(:,3)-posicao_z), 'b')
% grid minor
% xlabel('Tempo (s)')
% ylabel('Erro (m)')
% title('Erro na altitude')
% subplot(3,1,2)
% plot(t,(X_d(:,1)-posicao_x), 'g')
% grid minor
% xlabel('Tempo (s)')
% ylabel('Erro (m)')
% title('Erro na posição x')
% subplot(3,1,3)
% plot(t,(X_d(:,2)-posicao_y), 'r')
% grid minor
% xlabel('Tempo (s)')
% ylabel('Erro (m)')
% title('Erro na posição y')
%% Sinal de controle
% Sinal de controle individual de cada motor
Forces = zeros(length(Sinal_controle(:,1)), 4);
for i=1:(length(Sinal_controle(:,1)))
   [~, ~, Forces(i,:)] = F2W(Controle(i,:)');
end    
figure;
subplot(2,2,1)
plot(t, Vector_Tmax, '-.r', 'LineWidth',2); hold on;
plot(t, Vector_Tmin, '-.r', 'LineWidth',2); hold on;
plot(t,Forces(:,1), 'b', 'LineWidth',1); hold off;
grid minor
ylabel('F_1 (N)')
subplot(2,2,2)
plot(t, Vector_Tmax, '-.r', 'LineWidth',2); hold on;
plot(t, Vector_Tmin, '-.r', 'LineWidth',2); hold on;
plot(t,Forces(:,2), 'b', 'LineWidth',1); hold off;
grid minor
ylabel('F_2 (N)')
subplot(2,2,3)
plot(t, Vector_Tmax, '-.r', 'LineWidth',2); hold on;
plot(t, Vector_Tmin, '-.r', 'LineWidth',2); hold on;
plot(t,Forces(:,1), 'b', 'LineWidth',1); hold off;
grid minor
ylabel('F_3 (N)')
subplot(2,2,4)
plot(t, Vector_Tmax, '-.r', 'LineWidth',2); hold on;
plot(t, Vector_Tmin, '-.r', 'LineWidth',2); hold on;
plot(t,Forces(:,2), 'b', 'LineWidth',1); hold off;
grid minor
ylabel('F_4 (N)')
title('Thrust of each motor')
% 
% figure;
% subplot(4,1,1)
% plot(t,Controle(:,1), 'b', 'LineWidth',1); hold on;
% plot(t, 4*Vector_Tmax, '-.r', 'LineWidth',2); hold on;
% plot(t, 4*Vector_Tmin, '-.r', 'LineWidth',2); hold on;
% grid minor
% ylabel('u_1 (N)')
% title('Thrust')
% subplot(4,1,2)
% plot(t,Controle(:,2), 'b', 'LineWidth',1); hold on;
% plot(t, Vector_Mmax, '-.r', 'LineWidth',2); hold on;
% grid minor
% ylabel('u_2 (Nm)')
% title('Roll moment')
% subplot(4,1,3)
% plot(t,Controle(:,2), 'b', 'LineWidth',1); hold on;
% plot(t, Vector_Mmax, '-.r', 'LineWidth',2); hold on;
% grid minor
% ylabel('u_3 (Nm)')
% title('Pitch moment')
% subplot(4,1,4)
% plot(t,Controle(:,2), 'b', 'LineWidth',1); hold on;
% plot(t, Vector_MYawmax, '-.r', 'LineWidth',2); hold on;
% grid minor
% ylabel('u_4 (Nm)')
% title('Yaw moment')

%% Gráfico com Momento, Taxa (desejada vs real), ângulo (desejado vs real)
% figure;
% subplot(3,1,1)
% plot(t,Controle(:,2), 'b', 'LineWidth',1); hold on;
% plot(t, Moment_max, '-.r', 'LineWidth',2);
% grid minor
% ylabel('u_2 (Nm)')
% title('Moment')
% subplot(3,1,2)
% plot(t,rolagem, 'b', 'LineWidth',1)
% hold on
% plot(t,Ang_Target(:,1), '-.r', 'LineWidth',1)
% ylabel('\phi (rad)')
% title ('roll')
% legend('Simulated', 'Desired')
% grid minor
% subplot(3,1,3)
% plot(t,Todos_estados(:,10), 'b', 'LineWidth',1)
% hold on
% plot(t,Ang_Target(:,4), '-.r', 'LineWidth',1)
% ylabel('p (rad/s)')
% title('roll rate')
% legend('Simulated', 'Desired')
% grid minor
% sgtitle('Results Roll controller')
% figure;
% subplot(3,1,1)
% plot(t,Controle(:,3), 'b', 'LineWidth',1); hold on;
% plot(t, Moment_max, '-.r', 'LineWidth',2);
% grid minor
% ylabel('u_3 (Nm)')
% title('Moment')
% subplot(3,1,2)
% plot(t,arfagem, 'b', 'LineWidth',1)
% hold on
% plot(t,Ang_Target(:,2), '-.r', 'LineWidth',1)
% ylabel('\theta (rad)')
% title ('pitch')
% legend('Simulated', 'Desired')
% grid minor
% subplot(3,1,3)
% plot(t,Todos_estados(:,11), 'b', 'LineWidth',1)
% hold on
% plot(t,Ang_Target(:,5), '-.r', 'LineWidth',1)
% ylabel('q (rad/s)')
% title('pitch rate')
% legend('Simulated', 'Desired')
% grid minor
% sgtitle('Results Pitch controller')
% figure;
% subplot(3,1,1)
% plot(t,Controle(:,4), 'b', 'LineWidth',1); hold on;
% plot(t, MomentYaw_max, '-.r', 'LineWidth',2);
% grid minor
% ylabel('u_4 (Nm)')
% title('Moment')
% subplot(3,1,2)
% plot(t,guinada, 'b', 'LineWidth',1)
% hold on
% plot(t,Ang_Target(:,3), '-.r', 'LineWidth',1)
% ylabel('\psi (rad)')
% title ('yaw')
% legend('Simulated', 'Desired')
% grid minor
% subplot(3,1,3)
% plot(t,Todos_estados(:,12), 'b', 'LineWidth',1)
% hold on
% plot(t,Ang_Target(:,6), '-.r', 'LineWidth',1)
% ylabel('r (rad/s)')
% title('yaw rate')
% legend('Simulated', 'Desired')
% grid minor
% sgtitle('Results Yaw controller')
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