function [ F, M ] = controller(~, state, des_state, params)
%CONTROLLER  Controller for the planar quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
%   state.omega = [phi_dot]
%
%   des_state: The desired states are:
%   des_state.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
%   [y_ddot; z_ddot]
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls
%F = 0;
%M = 0;
%Kpz = 50;
Kpz = 50;
%Kvz = 40;
Kvz = 40;
%Kpy = 200;
Kpy = 250;
%Kvy = 150;
Kvy = 200;
%Kpphi = 30;
Kpphi = 35;
%Kvphi = 120;
Kvphi = 125;
% FILL IN YOUR CODE HERE
F = params.mass*(params.gravity + des_state.acc(2,:) + Kvz*(des_state.vel(2,:)-state.vel(2,:)) + Kpz*(des_state.pos(2,:) - state.pos(2,:)));
%phic = -0.5;
phic = -(1/params.gravity)*(des_state.acc(1,:) + Kvphi*(des_state.vel(1,:)-state.vel(1,:)) + Kpphi*(des_state.pos(1,:) - state.pos(1,:)));
M = params.Ixx*(0 + Kvy*(0-state.omega) + Kpy*(phic-state.rot));
%M = 0;
end

