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
 
kp_phi = 1000.01;
kv_phi = 0.001;
 
kp_theta = 1000.01;
kv_theta = 0.001;
 
kp_yaw = 0.1;
kv_yaw = 0.1;
 

%  Moment
 
% M = (params.I)*[(-kv_phi*state(10) + kp_phi*(phi_des - state(7))); 
%      (-kv_theta*state(11) + kp_theta*(theta_des - state(8))); 
%      (kv_yaw*(des_state(12) - state(12)) + kp_yaw*(psi_des - state(9)))];
M2 = params(3)*(-kv_phi*state(10) + kp_phi*(phi_des - state(7)));
M3 = params(4)*(-kv_theta*state(11) + kp_theta*(theta_des - state(8)));
M4 = params(5)*(kv_yaw*(des_state(12) - state(12)) + kp_yaw*(psi_des - state(9)));

% =======================================================================

end
