function [omega, omega_square, Forces] = F2W(u)
%F2W Summary of this function goes here
%   Detailed explanation goes here
 kf = 1.4351e-5;  % constante de empuxo
 km = 2.4086e-7; % constante de torque
 L = 0.26;       % distância ao CG 
 T = [kf,kf,kf,kf;
      0, L*kf,0,-L*kf;
      -L*kf,0,L*kf,0;
      -km,km,-km,km];
 T_inv = T^(-1); 
 omega_square = T_inv*u;
 Forces = kf*omega_square;
 omega = zeros(4,1);
 for i=1:length(omega_square)
        omega(i,:) = sqrt(abs(omega_square(i)));
 end
end

