function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yaw_dot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls


% =================== Your code goes here ===================

% Thurst
Kdz = 10;
Kpz = 100;
Kdy = 10;
Kpy = 100;
Kdx = 10;
Kpx = 100;
F = params.mass*(params.gravity+des_state.acc(3)) + params.mass*(Kdz*(des_state.vel(3) - state.vel(3)) + Kpz*(des_state.pos(3)-state.pos(3)));
R1des = des_state.acc(1) + Kdx*(des_state.vel(1)-state.vel(1)) + Kpx*(des_state.pos(1) - state.pos(1));
R2des = des_state.acc(2) + Kdy*(des_state.vel(2)-state.vel(2)) + Kpy*(des_state.pos(2) - state.pos(2));
% Moment
M = zeros(3,1);
Kpphi = 10;
Kdphi = 0.10;
Kptheta = 10;
Kdtheta = 0.10;
Kppsi=5;
Kdpsi =0.05;

thetades=(1/params.gravity)*(R1des*cos(des_state.yaw) + R2des*sin(des_state.yaw));
phides = (1/params.gravity)*(R1des*sin(des_state.yaw) - R2des*cos(des_state.yaw));
psides = des_state.yaw;

uphi = Kpphi*(phides - state.rot(1)) + Kdphi*(0 - state.omega(1));
utheta = Kptheta*(thetades - state.rot(2)) + Kdtheta*(0 -state.omega(2));
upsi = Kppsi*(psides - state.rot(3)) + Kdpsi*(des_state.yawdot - state.omega(3));

M = [uphi; utheta; upsi];
% =================== Your code ends here ===================

end
