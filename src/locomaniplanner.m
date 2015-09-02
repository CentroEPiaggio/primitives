% locomaniplanner
clear all; clear import; close all; clc;

run utils/startup_lmp.m;

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
N_sample_max = 100; % max number of samples
% main loop
for ii=1:N_sample_max
    ii
    %waitforbuttonpress
    %% sample a point, randomly, in Chi0
    x_rand = Chi0.sample;
    plot(x_rand(1),x_rand(2),'x','linewidth',2)
    %% find the nearest point, in Chi0
    % convert nodes in trees from cells to matrix
    points_mat = cell2mat(T.Node');
    % find nearest point
    idx_nearest = knnsearch(points_mat',x_rand'); % TODO: this can be replaced by a search in the least-cost sense, employing the primitive's cost_table
    % TODO: how to represent the space? I suggest sth like sparse matrices
    % with NaNs for non-intersecting (or not yet sampled) dimensions
    %     keyboard
    %waitforbuttonpress
    % TODO: valutare il miglior parametro per muoversi in Chi0
    
    %% add the new node to the tree
    T = T.addnode(idx_nearest,x_rand);
    % draw arc between newest sample and nearest % TODO: pick best q in the
    % primitive
    x_nearest = T.get(idx_nearest);
    line([x_nearest(1) x_rand(1)],[x_nearest(2) x_rand(2)],'color','red');
    
    % TODO: check if other primitives are available from the new point
    
    %% check if other dimensions can be activated from the newest point (x_rand)
    prim_cost = zeros(P.nnodes,1); % cost vector, to choose between different primitives the cheaper one
    for jj=1:P.nnodes % start looking between all available primitives
        prim = P.get(jj); % prim is the current primitive
        if prim.chi.P.contains(x_rand) % is the new point contained here? % TODO: manage search over Chi0
            %              disp('beccato!')
            % TODO: sample in
            %             x_rand_new = sample(prim.chi.P\Chi0.P)
            % evaluate best parameter for the primitive
            %              prim_cost(jj) = prim.findbest(x_nearest,x_rand);
            disp(['beccato! Potrei usare ' prim.getName ' con un costo ' num2str(prim_cost(jj))]);
        else
            disp('ahaha! non hai detto la parola magica!')
            prim_cost(jj) = Inf;
        end
    end
    [~,idx_p_opt] = min(prim_cost);
    if prim_cost(idx_p_opt) == Inf
        disp(['nessuna primitiva con costo finito disponibile']);
    else
        prim_opt = P.get(idx_p_opt);
        disp(['scelgo la primitiva ' prim_opt.getName ' con un costo ' num2str(prim_cost(idx_p_opt))])
    end
    %      sample(P.ge
    
    %     drawnow
    %     pause(0.01)
end
return

% disp(T.tostring)