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

x_I = [0  ;0  ;NaN;NaN]; % initial state. Position and speed of cart are both zeros
x_G = [NaN;NaN;NaN;  1]; % goal state. Button shall be pressed
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
N_sample_max = 20; % max number of samples

% empty search graph
G = sparse(1,1,0);
E = cell(1,1);
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
    [T,G,E] = localRRTstar(Chi,Ptree,x_rand,T,G,E);
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
%% simulate the optimal plan that has been found
source_node = 1;
goal_node = 21; % just to try
% make the sparse matrix square
sizeG = size(G);
[~,shorterDim]=min(sizeG);
G(sizeG(shorterDim)+1:max(sizeG),:)=0;
% show the graph
h = view(biograph(G,[],'ShowWeights','on'));
% find the shortest path
[dist,path,pred] = graphshortestpath(G,source_node,goal_node);
% draw shortest path
set(h.Nodes(path),'Color',[1 0.4 0.4])
edges = getedgesbynodeid(h,get(h.Nodes(path),'ID'));
set(edges,'LineColor',[1 0 0])
set(edges,'LineWidth',1.5)
%% obtain plan as the shortest path in the graph
opt_plan = tree;
prim_name = 'Standby';
prim_params = 0;
pi_I = struct('primitive',prim_name, ...
    'primitive_q',prim_params);
opt_plan = opt_plan.addnode(0,pi_I);
for ii=2:length(path)
    idx_child = path(ii);
    idx_parent = T.Parent(idx_child); % because tree class uses 0 as starting index
    opt_plan=opt_plan.addnode(ii-1,E{idx_parent,idx_child});
end

%% go to test the plan
% assemble the optimal plan
Tend = 10; % TODO porcata. Il tempo va parametrizzato.
Ts = 0.01;
run_filepath = '../example/';
prim_filepath = [run_filepath 'prim/'];
% init
q_reference = [0;0;0];
% loop
for ii=1:length(opt_plan.Node)
    ii
    switch opt_plan.Node{ii}.primitive
        case 'Muovi'
            xi = opt_plan.Node{ii}.primitive_q(1);
            xf = opt_plan.Node{ii}.primitive_q(2);
            vi = opt_plan.Node{ii}.primitive_q(3);
            vf = opt_plan.Node{ii}.primitive_q(4);
            primitive_muovi_params = struct('name','muovi',    ...
                'xi',xi,            ...
                'xf',xf,            ...
                'vi',vi, ...
                'vf',vf, ...
                'Tend',Tend,        ...
                'Ts',Ts,            ...
                'xf_vec_len',1, ...
                'vx0_vec_len',1,  ...
                'vxf_vec_len',1, ...
                'filepath',prim_filepath ...
                );
            [time,traj_x_cart]=gen_primitives_muovi(primitive_muovi_params);
            traj_y_cart = zeros(size(traj_x_cart));
            q_reference_add = [q_reference(1,end)+time(:)';
                traj_x_cart(:)';
                traj_y_cart(:)'];
            q_reference = [q_reference, q_reference_add];
        otherwise
            disp('standby');
    end
end
save([run_filepath 'runna.mat'],'q_reference');
save([run_filepath 'rsim_tfdata.mat'],'q_reference');
Tend = q_reference(1,end);
q0 = [0;deg2rad(90);2];
qp0 = [0;0;0];
qref0 = q0;
ic = struct('q0',q0,'qp0',qp0,'qref0',qref0);
gen_ic(ic);
m1 = 100;
m2 = 1;
m3 = 100;
masses=[m1;m2;m3];
nq=length(q0);
runstr = [run_filepath, 'modello -f rsim_tfdata.mat=' run_filepath 'runna.mat -p ' run_filepath 'params_steering.mat -o ' run_filepath 'optimal.mat -v -tf ',num2str(Tend)];
[status, result] = system(runstr);
if status ~= 0, error(result); end
