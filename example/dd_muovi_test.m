close all
clear all
clc

run ../src/utils/startup_lmp.m;
import primitive_library.*;

%%

cost_table = rand(10,3);

xmin = -2;
xmax = 25;
vmin = -5;
vmax = +5;
vmin = -0.01;
vmax = +1;
dimensioni = [1 1 0 0];
initial_extend = [0 0 NaN NaN];

Muovi = DD_move([xmin vmin; xmin vmax; xmax vmax; xmax vmin],[1 0],cost_table,'DD_Muovi',dimensioni,initial_extend);

%%

% x,y,theta,v
z_start=[0 0 0 0];
z_end=[1 1 pi/2 0];

[feasible,cost,q,x,time] = Muovi.steering(z_start,z_end)