% locomaniplanner
clear all; clear import; close all; clc;
init % run init script just once to load all libraries. Customize this. TODO: make this  smarter

addpath(pwd) % add current dir to path
addpath('cprnd/');

import primitive_library.*;

x_I = [0;0]; % initial state
x_G = [2;2]; % goal state
% initialize an empty tree
T = tree;
% add inial state to the tree
T = T.addnode(0,x_I);
% set initial image space
angolo = pi/4;
Chi0 = Imagespace(([cos(angolo) sin(angolo);-sin(angolo) cos(angolo)]*[-1 -1;-1 1;1 -1; 1 1]'*1)');

Chi0.P.plot('color','lightgreen');hold on;     % plot search region (piano)
axis equal;
plot(x_I(1),x_I(2),'kx','linewidth',2) % plot initial point

% algorithm parameters
N_sample_max = 1000; % max number of samples
for ii=1:N_sample_max
    % sample a point, randomly, in Chi0
    x_rand = sample(Chi0);
    plot(x_rand(1),x_rand(2),'x','linewidth',2)
    % find the nearest point, in Chi0
    % convert nodes in trees from cells to matrix
    points_mat = cell2mat(T.Node);
    idx = knnsearch(points_mat',x_rand')
    % TODO: how to represent the space? I suggest with sparse matrices
    keyboard
end
return

% disp(T.tostring)