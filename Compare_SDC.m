%---------------- Comparação de parametrização SDC -----------------------%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear; clc;
close all;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Inicializar uma matriz para armazenar os dados
% result = zeros(100, 3);  % 100 linhas x 3 colunas
% 
% for i = 1:100
%     % Nome do arquivo
%     filename = sprintf('DATA_J_alpha/Test_alpha1_%d.mat', i);
%     
%     % Carregar o arquivo
%     data = load(filename);
%     
%     % Supondo que os dados estão na variável chamada 'data'
%     % Adicionar a linha correspondente à matriz result
%     result(i, :) = data.J_alpha;  
% end
% 
% % Converter a matriz para uma tabela
% %resultTable = array2table(result, 'VariableNames', {'Col1', 'Col2', 'Col3'});
% 
% % Salvar a tabela em um arquivo .mat (opcional)
% save('DATA_J_alpha/resultado_final.mat', 'result');

flag_alpha = 0;
%% Caso de variar apenas alpha1
if flag_alpha
% Carregando as diferentes funções de custo
% a0  = load("DATA_J_alpha/SDREv4_alpha0.mat");
% a1  = load("DATA_J_alpha/SDREv4_alpha01.mat");
% a2  = load("DATA_J_alpha/SDREv4_alpha02.mat");
% a3  = load("DATA_J_alpha/SDREv4_alpha03.mat");
% a4  = load("DATA_J_alpha/SDREv4_alpha04.mat");
% a5  = load("DATA_J_alpha/SDREv4_alpha05.mat");
% a6  = load("DATA_J_alpha/SDREv4_alpha06.mat");
% a7  = load("DATA_J_alpha/SDREv4_alpha07.mat");
% a8  = load("DATA_J_alpha/SDREv4_alpha08.mat");
% a9  = load("DATA_J_alpha/SDREv4_alpha09.mat");
% a10 = load("DATA_J_alpha/SDREv4_alpha1.mat");
% Jalpha = [a0.J_alpha; a1.J_alpha; a2.J_alpha; a3.J_alpha; a4.J_alpha; a5.J_alpha; a6.J_alpha; a7.J_alpha; a8.J_alpha; a9.J_alpha; a10.J_alpha]; % Concatena tudo em uma tabela

    alpha = 0:0.01:0.99; % Vetor de alpha_1

    figure; 
    subplot(3,1,1)
    plot(alpha, result(:,1), 'k', 'LineWidth',1); grid minor;
    ylabel('J [-]')
    xlabel('\alpha_1 [-]')
    legend('J')
    title('Performance Index')
    subplot(3,1,2)
    plot(alpha, result(:,2), 'r', 'LineWidth',1); grid minor;
    ylabel('J_e [-]')
    xlabel('\alpha_1 [-]')
    legend('J_e')
    title('Performance States Error')
    subplot(3,1,3)
    plot(alpha, result(:,3), 'b', 'LineWidth',1); grid minor;
    ylabel('J_u [-]')
    xlabel('\alpha_1 [-]')
    legend('J_u')
    title('Performance Control Effort')
else
    a1= 0:0.1:1; a2= 0:0.1:1;
    % Supondo que alpha1, alpha2, e Z já são definidos
    [Alpha1, Alpha2] = meshgrid(a1, a2);  % Criação da malha (se necessário)
    J_matrix = zeros(11); Je_matrix = zeros(11); Ju_matrix = zeros(11);
    for i=1:11
        for j=1:11
            J_matrix(i,j) = J_Final(j,3);
            Je_matrix(i,j) = J_Final(j,4);
            Ju_matrix(i,j) = J_Final(j,5);
        end
    end
    %J_matrix = reshape(J_Final(:,3), length(a1), length(a2));
    % Plotar a superfície
    figure;
    surf(Alpha1, Alpha2, J_matrix);
    
    % Adicionar títulos e rótulos
    title('Gráfico de Superfície');
    xlabel('\alpha_1');
    ylabel('\alpha_2');
    zlabel('J');

    % Ajustar a aparência
    shading interp;
    colorbar;
    % Verifica a melhoria do custo em relação ao erro estado
    %Je_matrix = reshape(J_Final(:,4), length(a1), length(a2));
    figure;
    surf(Alpha1, Alpha2, Je_matrix);
    
    % Adicionar títulos e rótulos
    title('Superfície parcela Je');
    xlabel('\alpha_1');
    ylabel('\alpha_2');
    zlabel('Je');
    % Ajustar a aparência
    shading interp;
    colorbar;
    % Verifica a melhoria do custo em relação ao erro estado
    %Ju_matrix = reshape(J_Final(:,5), length(a1), length(a2));
    figure;
    surf(Alpha1, Alpha2, Ju_matrix);
    
    % Adicionar títulos e rótulos
    title('Superfície parcela Ju');
    xlabel('\alpha_1');
    ylabel('\alpha_2');
    zlabel('Ju');
    % Ajustar a aparência
    shading interp;
    colorbar;

end

