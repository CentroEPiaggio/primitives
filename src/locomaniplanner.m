% locomaniplanner
clear all; clear import; close all; clc;

% init % run init script just once to load all libraries. Customize this. TODO: make this  smarter
% return

import primitive_library.*;

load_primitive_tree; % builds a list with all available primitives

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

% plotta gli spazi immagini delle primitive
for jj=1:P.nnodes
         prim = P.get(jj);
         prim.chi.P.plot;
 end

% algorithm parameters
N_sample_max = 1000; % max number of samples
for ii=1:N_sample_max
    % sample a point, randomly, in Chi0
    x_rand = sample(Chi0);
    plot(x_rand(1),x_rand(2),'x','linewidth',2)
    % find the nearest point, in Chi0
    % convert nodes in trees from cells to matrix
    points_mat = cell2mat(T.Node');
    idx_nearest = knnsearch(points_mat',x_rand');
    % TODO: how to represent the space? I suggest sth like sparse matrices
    % with NaNs for non-intersecting (or not yet sampled) dimensions
%     keyboard
    waitforbuttonpress
    
    % add new node to the tree
    T = T.addnode(idx_nearest,x_rand);
    % draw arc between newest sample and nearest
    x_nearest = T.get(idx_nearest);
    line([x_nearest(1) x_rand(1)],[x_nearest(2) x_rand(2)],'color','red');
    
    % TODO: check if other primitives are available from the new point
     for jj=1:P.nnodes
         prim = P.get(jj);
         if prim.chi.P.contains(x_rand) % TODO: manage search over Chi0
             disp('beccato!')
         else
             disp('ahaha! non hai detto la parola magica!')
         end
     end
%     drawnow
%     pause(0.01)
end
return

% disp(T.tostring)