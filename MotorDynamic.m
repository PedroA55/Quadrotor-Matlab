function [U_real, omega_real] = MotorDynamic(u)
%MOTORDYNAMIC Summary of this function goes here
%   entradas: A��o de controle U = [u1, u2, u3, u4]
%   sa�da: Nova a��o de controle Ureal = [u1, u2, u3, u4]

 kf = 1.4351e-5;  % constante de empuxo
 km = 2.4086e-7;  % constante de torque
 L = 0.26;        % dist�ncia ao CG 
 ke = 0.024;      % constante de for�a eletromotriz
 ktau = 0.024;    % constante de torque
 Rmot = 0.2;      % Resist�ncia da armadura
 Jr = 5e-5;       % Momento de in�rcia do rotor (assume-se que seja igual para todos os motores)
 w_min = 296; % Limite inferior para rota��o em rad/s
 w_max = 657; % Limite superior para rota��o em rad/s
 T = [kf,kf,kf,kf;
      0, L*kf,0,-L*kf;
      -L*kf,0,L*kf,0;
      -km,km,-km,km];
 T_inv = T^(-1); 
 omega_square = T_inv*u;
 omega = zeros(4,1);
 for i=1:length(omega_square)
        omega(i,:) = sqrt(abs(omega_square(i)));
        if omega(i,:)< w_min
            omega(i,:) = w_min;
        end
        if omega(i,:)> w_max
            omega(i,:) = w_max;
        end
 end
    
% Modelo din�mico do motor - 1 ordem

omega_real = zeros(4,1);
for i=1:length(omega_real)
   omega_real(i,:) = omega(i,:)^2; 
end
U_real = T*omega_real;

end

