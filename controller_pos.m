function [F, phi_des, theta_des, psi_des, I] = controller_pos(state, des_state, acc, params, I, Ts)
%Controlador para a posição do quadrotor (Controlador PID)
%
%   state: Estados reais do quadrirrotor
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls

% =================== Ganhos do controle de posição ===================

% Ganhos
kp_x = 4*10^(-8);
ki_x = 0;
kd_x = 10;

kp_z = 1*10^(-7);
ki_z = 11*10^(0); % erro não converge. Fica próximo mas não converge
kd_z = 3.5*10^(1); %
 
kp_y = 4*10^(-8);
ki_y = 2*10^(-6);
kd_y = 10;

% Ação Integral
Ix = I(1) + (des_state(1) - state(1))*Ts;
Iy = I(2) + (des_state(2) - state(2))*Ts;
Iz = I(1) + (des_state(3) - state(3))*Ts;
I = [Ix Iy Iz];

% Acelerações

a1_des = acc(1) + kd_x*(des_state(4) - state(4)) + kp_x*(des_state(1) - state(1)) + ki_x*Ix;
a2_des = acc(2) + kd_y*(des_state(5) - state(5)) + kp_y*(des_state(2) - state(2)) + ki_y*Iy;
a3_des = acc(3) + kd_z*(des_state(6) - state(6)) + kp_z*(des_state(3) - state(3)) + ki_z*Iz;

% Empuxo total
F = params(2)*(params(1) + a3_des);

% Ângulos

phi_des = (a1_des*sin(des_state(9)) - a2_des*cos(des_state(9)))/params(1);
theta_des = (a1_des*cos(des_state(9)) + a2_des*sin(des_state(9)))/params(1);
psi_des = des_state(9);


% =======================================================================

end
