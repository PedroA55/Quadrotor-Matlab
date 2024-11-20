% Loop para avaliar a mudança de matrizes A(x, alpha)
alpha1 = 0;
alpha2 = 0;
alpha = [alpha1 alpha2 1];
J_Final = [0 0 0 0 0];
n = 5;
for s=1:n+1
    for k=1:n+1
        %% Roda o modelo não linear
        [x, I,acc, U, x_ant, wd] = ResetSimulation(xCI);
        for i=2:(length(t)-2)
            %-------------------- aceleração de posição --------------------------%
            ddx = (X_d(i,4) - X_d(i-1,4))/Ts; ddy = (X_d(i,5) - X_d(i-1,5))/Ts; ddz = (X_d(i,6) - X_d(i-1,6))/Ts; 
            acc(i,:) = [ddx, ddy, ddz]; % será que é a melhor forma de derivar ?
        end
        acc0 = [X_d(1,4), X_d(1,5), X_d(1,6)]; % primeiro elemento de aceleração
        acc = [acc0; acc]; %Agora tenho um vetor de mesmo tamanho mas com uma linha igual a zero
        for i=1:(length(t)-1)
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
            %PID para posição
            [u1, phi_des, theta_des, psi_des, I] = controller_pos(x, X_d(i,:), acc(i,:), params, I, Ts);    
            %-------- Gerando a trajetória de referência para a atitude ----------%
            dphi = (phi_des - x_ant(7))/Ts; dtheta = (theta_des - x_ant(8))/Ts;
            dpsi = (psi_des - x_ant(9))/Ts;
            usa_filtro_PB = 1;
            %---------------- Filtrando as derivadas númericas -------------------%
            if usa_filtro_PB
                [dphi, dtheta, dpsi] = Filtro_PB(dphi, dtheta, dpsi, Ts);
            end    
            z = [phi_des, theta_des, psi_des, dphi, dtheta, dpsi];
            %------------------- Controladores de atitude ------------------------%
            [u2, u3, u4] = controller_SDREv4(x,wd,z, Qv4, Rv4, Cv4, params, alpha);
            %-------------------- Sinal de Controle Final ------------------------%
            U = [u1, u2, u3, u4]';
            %------------------------ Modelo dos motores -------------------------%
            if incluir_din_motor
                [U, omega_real] = MotorDynamic(U);
                omega_motor(i+1,:) = omega_real';
            end
            %------------------------ Planta não linear --------------------------%
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

        posicao_x = Dados_posicao(:,1);
        posicao_y = Dados_posicao(:,2);
        posicao_z = Dados_posicao(:,3);
        rolagem = Dados_atitude(:,1);
        arfagem = Dados_atitude(:,2);
        guinada = Dados_atitude(:,3);
        Controle = [[4*Tmin, 0, 0, 0]; Sinal_controle(2:end,:)];
        % --------------- Calculo do Indice de Desempenho --------------------%
        pqr_des = Angle2pqr_Target(Ang_Target,t);
        J_t = cost_function(pqr_des, Monitorar(:,1:3),Controle(:,2:4),Qv4,Rv4,t);
        Je = trapz(J_t(:,2));
        Ju = trapz(J_t(:,3));
        J = Je + Ju;
        % ------------------- Montando o vetor de Dados de J ---------------- %
        J_alpha = [alpha1, alpha2, J, Je, Ju];
        % Se eu quisesse salvar os dados em cada iteração
        %filename = sprintf("DATA_J_alpha/Test_JFinal_%d.mat", k);
        J_Final = [J_Final; J_alpha];

        % ----------------- Incrementa o valor de alpha1 --------------------- %
        alpha1 = alpha1 + (1/n);
        alpha = [alpha1 alpha2 1];
    end
    alpha2 = alpha2 + (1/n);
    alpha1 = 0;
end

% ------------------- Salvando os Dados de J ------------------------ %
% Retira a primeira linha de zeros que foi feita na inicialização
J_Final = J_Final(2:end,:);
filename = sprintf("DATA_J_alpha/Test5_JFinal.mat");
save(filename, 'J_Final');