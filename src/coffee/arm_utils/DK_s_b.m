% DK_s_b coordinate transformation from moving base frame b to shoulder
% base s
% input:
% q: [x,y,z], where
% x: location in meters
% y: location in meters
% z: location in meters
function [p0,A_b_0,o,R] = DK_s_b(q)
x = q(1);
y = q(2);
z = q(3);
theta = 0; % DH-parameter in radians

alpha = 0; % DH-parameter
d = z; % DH-parameter

pb = [x;y;0;1];
%Forward Kinematics
A_b_0 =  [  cos(theta)  , -sin(theta)*cos(alpha)    , sin(theta)*sin(alpha)     , x;
            sin(theta)  , cos(theta)*cos(alpha)     , -cos(theta)*sin(alpha)    , y;
            0           , sin(alpha)                , cos(alpha)                , d;
            0           , 0                         , 0                         , 1];

p0 = A_b_0*pb;

o = A_b_0(1:3,4);
R = A_b_0(1:3,1:3);