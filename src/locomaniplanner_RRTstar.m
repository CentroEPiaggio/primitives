% locomaniplanner
clear all; clear import; close all; clc;

run utils/startup_lmp.m;

import primitive_library.*;

load_primitive_tree; % builds a list with all available primitives
% plotta gli spazi immagini delle primitive (tutti schiacciati nello stesso
% piano)
for jj=1:Ptree.nnodes
    prim = Ptree.get(jj);
    prim.chi.P.plot;
    hold on
end

x_I = [0;0;NaN;NaN]; % initial state. Position and speed of cart are both zeros
x_G = [NaN;NaN;1]; % goal state. Button shall be pressed
% initialize an empty tree
T = tree;
% add inial state to the tree
T = T.addnode(0,x_I);

% set initial image space
% angolo = pi/4;
% Chi0 = Imagespace(([cos(angolo) sin(angolo);-sin(angolo) cos(angolo)]*[-1 -1;-1 1;1 -1; 1 1]'*1)');
Chi0 = Ptree.Node{1}.chi; % conventionally in node{1} we have the chi0 space
dimChi0 = Chi0.P.Dim;

figure
Chi0.P.plot('color','lightgreen');hold on;     % plot search region (piano)
axis equal;
plot(x_I(1),x_I(2),'kx','linewidth',2) % plot initial point

% algorithm parameters
N_sample_max = 10; % max number of samples

%%
% main loop
for ii=1:N_sample_max
    ii
    %waitforbuttonpress
    %% sample a point in Chi0.
    x_rand = Chi0.sample;
    plot(x_rand(1),x_rand(2),'x','linewidth',2)
    
    %% forall primitives living in Chi0
    Chi = Chi0;
    T = localRRTstar(Chi,Ptree,x_rand,T);
%     keyboard
%     %% check if other dimensions can be activated from the newest point (x_rand)
%     prim_cost = zeros(P.nnodes,1); % cost vector, to choose between different primitives the cheaper one
%     for jj=1:P.nnodes % start looking between all available primitives
%         prim = P.get(jj); % prim is the current primitive
%         if prim.chi.P.contains(x_rand) % is the new point contained here? % TODO: manage search over Chi0
%             %              disp('beccato!')
%             % TODO: sample in
%             %             x_rand_new = sample(prim.chi.P\Chi0.P)
%             % evaluate best parameter for the primitive
%             %              prim_cost(jj) = prim.findbest(x_nearest,x_rand);
%             disp(['Found primitive ' prim.getName ' with cost: ' num2str(prim_cost(jj))]);
%         else
%             disp('No primitives found')
%             prim_cost(jj) = Inf;
%         end
%     end
%     [~,idx_p_opt] = min(prim_cost);
%     if prim_cost(idx_p_opt) == Inf
%         disp(['nessuna primitiva con costo finito disponibile']);
%     else
%         prim_opt = P.get(idx_p_opt);
%         disp(['scelgo la primitiva ' prim_opt.getName ' con un costo ' num2str(prim_cost(idx_p_opt))])
%     end

end
return

disp(T.tostring)