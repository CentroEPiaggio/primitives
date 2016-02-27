% DK_b_0 coordinate transformation from inertial frame 0 to moving base
% frame b
% input:
% q: [x,y,theta,v,w], where
% x: location in meters
% y: location in meters
% theta: vehicle orientation in radians
% v: linear speed in m/s
% w: angular speed in rad/s
function [p0,A_b_0,o,R] = DK_b_0(q)


x = q(1);
y = q(2);
theta = q(3); % DH-parameter in radians
if length(q) >= 4
    v = q(4);
end
if length(q) == 5
    w = q(5);
end

a = sqrt(x^2+y^2); % DH-parameter in m

alpha = 0; % DH-parameter
d = 0; % DH-parameter

pb = [x;y;0;1];
%Forward Kinematics
A_b_0 =  [  cos(theta)  , -sin(theta)*cos(alpha)    , sin(theta)*sin(alpha)     , x;
            sin(theta)  , cos(theta)*cos(alpha)     , -cos(theta)*sin(alpha)    , y;
            0           , sin(alpha)                , cos(alpha)                , d;
            0           , 0                         , 0                         , 1];

p0 = A_b_0*pb;

o = A_b_0(1:3,4);
R = A_b_0(1:3,1:3);