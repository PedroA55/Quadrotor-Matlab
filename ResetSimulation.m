function [x, I,acc, U, x_ant, wd] = ResetSimulation(x0)
    x=x0;
    x_ant = x;
    wd = 0;
    U = [0 0 0 0]';
    acc = [0 0 0]; % Acelera��o de posi��o
    I = [0 0 0]; % Termo integrativo do controle de posi��o
end