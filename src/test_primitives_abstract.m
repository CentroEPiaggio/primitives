clear all; close all; clc;

import primitive_library.*;

xmin = -2; % it was 0, now it is -10 to allow primitives that start by going backwards and then move onwards
xmax = 25;
vmin = -5;
vmax = +5;
cost_table = rand(10,3);



dimensioni = [1 1 0 0]; % only sample in x and v cart.
default_extend = [0 0 NaN NaN];

x = Move([xmin vmin; xmin vmax; xmax vmax; xmax vmin],[1 0],cost_table,'Muovi',dimensioni,default_extend);


