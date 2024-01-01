function [M2, M3, M4] = controller_att(phi_des, theta_des, psi_des, state, des_state, params)
%Controlador para a posição do quadrotor (Controlador PD)
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

% =================== Ganhos do controle de atitude ===================

% Ganhos
 
kp_phi = 35;
kv_phi = 1.5*10^(1); % Utilize kv = 35 para a traj_estab_Atitude
 
kp_theta = 3.5*10^(1);
kv_theta = 1.5*10^(1); % Utilize kv = 35 para a traj_estab_Atitude
 
kp_yaw = 10;
kv_yaw = 10;
 

%  Moment
 
% M = (params.I)*[(-kv_phi*state(10) + kp_phi*(phi_des - state(7))); 
%      (-kv_theta*state(11) + kp_theta*(theta_des - state(8))); 
%      (kv_yaw*(des_state(12) - state(12)) + kp_yaw*(psi_des - state(9)))];
M2 = (params(3)/params(7))*(-kv_phi*state(10) + kp_phi*(phi_des - state(7)));
M3 = (params(4)/params(7))*(-kv_theta*state(11) + kp_theta*(theta_des - state(8)));
M4 = (params(5)/params(7))*(kv_yaw*(des_state(12) - state(12)) + kp_yaw*(psi_des - state(9)));

% =======================================================================

end
