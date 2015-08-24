clear all; close all; clc;

nq = 3;
q0 = zeros(nq,1);
q0(3) = 0.1; % altrimenti diventa singolare
qp0 = zeros(nq,1);
q0 = rand(nq,1)
q0(2)=0;
q0(3)=1;
% qp0 = rand(nq,1)
% q0 = [1,deg2rad(45),2];

% parameters
m1 = 100;
m2 = 1;
m3 = 100;
masses=[m1;m2;m3];