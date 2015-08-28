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

% traiettorie carrello
xi=q0(1);
xf=1;
T = [0 1];
[a5,a4,a3,a2,a1,a0] = createTraj5(xi,xf,0,0,0,0,T(1),T(2));
coeff=[a5,a4,a3,a2,a1,a0];
coeffp=polyder(coeff);
tt=T(1):0.01:T(2);
q_cart = polyval(coeff,tt-T(1));
qp_cart = polyval(coeffp,tt-T(1));
% sampling time for simulation (discrete time)
Ts = 0.001;
% simulation total time
Tend = T(end);
%
tt_d = tt(1):Ts:tt(end);
qp_cart_d = interp1(tt,qp_cart(:),tt_d);
% correct format for .mat loading:
% [t(1) t(2) ... t(N);
% [y(1) y(2) ... y(N)];
q_reference = [tt_d(:)';qp_cart_d(:)'];
save primitiva_muovi.mat q_reference;


%% loops!
% first loop: between every 
% possible reference value
% for the primitive MUOVI
!./modello
load('modello.mat');
plottini