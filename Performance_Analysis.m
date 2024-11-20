% Performance Analysis
% Separa os dados novamente (Talvez essa etapa não seja necessária no futuro)
% t             = DATA(:,1);
% Todos_estados = DATA(:,2:13);
% X_d           = DATA(:,14:25);
% Angle_Target  = DATA(:,26:31);
% Controle      = DATA(:,32:35);
% Monitorar     = DATA(:,36:44);

%Teste da função de pqr_des
pqr_des = Angle2pqr_Target(Ang_Target,t);
pqr_real = Monitorar(:,1:3);
%% Avaliando a derivada numérica
figure;
subplot(3,1,1)
plot(t,Ang_Target(:,4), 'k', 'LineWidth',2); grid minor;
ylabel('\phi [rad/s]');
xlabel('t [s]')
title('\phi')
subplot(3,1,2)
plot(t,Ang_Target(:,5), 'b', 'LineWidth',2); grid minor;
ylabel('\theta [rad/s]');
xlabel('t [s]')
title('\theta')
subplot(3,1,3)
plot(t,Ang_Target(:,6), 'r', 'LineWidth',2); grid minor;
ylabel('\psi [rad/s]');
xlabel('t [s]')
title('\psi')


%% Valor de J - índice de desempenho
J_t = cost_function(pqr_des, pqr_real,Controle(:,2:4),Qv4,Rv4,t);
figure; 
plot(t,J_t(:,1),'b', 'LineWidth',2); grid minor;
title('Índice de Desempenho (J_{u^*})')
xlabel('t [s]')
ylabel('J_u [-]')
legend('J_{u^*}')
%Valor total de cada componente da função custo
Je = trapz(J_t(:,2));
Ju = trapz(J_t(:,3));
J = Je + Ju;
Tabela1 = table(J,Je,Ju,'VariableNames',{'J', 'Je', 'Ju'});
disp(Tabela1);

%% Erro em relação a referência
Erros = Err_cal(pqr_des, pqr_real,Controle(:,2:4),Ts);

Tabela2 = table(Erros(:,1),Erros(:,2),Erros(:,3), Erros(:,4), Erros(:,5),'RowNames',{'p','q', 'r'},'VariableNames',{'ISE', 'IAE', 'ITSE', 'ITAE', 'Uctrl'});
disp(Tabela2);

%% Dados finais da Análise
% Dados da análise
DATA_Analysis = [t, J_t];
J_alpha = [J, Je, Ju];
% TabelaFinal = Tabela1;
save("DATA_J_alpha/Test1_alpha1.mat","J_alpha")
%save("DATA_Analysis/SDREv4_alpha09.mat","DATA_Analysis")