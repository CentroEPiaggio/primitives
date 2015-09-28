% locomaniplanner
% clear all; clear import; close all; clc;

verbose = 1;

run utils/startup_lmp.m;

import primitive_library.*;

% [Ptree,Chi0]=InitializePrimitives(); % builds a list with all available primitives
InitializePrimitives; % builds Ptree, which is a list with all available primitives, and Chi0: which is the common space

% actually instead of NaN we could use a value. Why is it better to use
% NaN? We'll see.
z_init = [0  ;0  ;NaN;NaN]; % initial state. Position and speed of cart are both zeros
z_goal = [NaN;NaN;NaN;  1]; % goal state. Button shall be pressed
z_goal = [17;   0;NaN;NaN]; % goal state for debug

[T,G,E] = InitializeTree();
[T,G,E] = InsertNode(0,z_init,T,G,E,[],0,0); % add first node

fig_points = 2; % figure handle to plot the sampled points and their connections (i.e. graph vertices and edges)
fig_trajectories = 3; % figure handle to plot the sampled points and their trajectories
figure(fig_points)
Chi0.P.plot('color','lightgreen','alpha',0.5);hold on;     % plot search region (piano)
axis equal;
plot(z_init(1),z_init(2),'go','linewidth',4) % plot initial point
plot(z_goal(1),z_goal(2),'ro','linewidth',4) % plot initial point
figure(fig_trajectories)
Chi0.P.plot('color','lightgreen','alpha',0.5);hold on;     % plot search region (piano)
axis equal;
plot(z_init(1),z_init(2),'go','linewidth',4) % plot initial point
plot(z_goal(1),z_goal(2),'ro','linewidth',4) % plot initial point

% define obstacles
Obstacles = tree;
speed_limit_bottom = -1;
speed_limit_top = 1;
speed_limit_entry = 10;
speed_limit_exit = 15;
obstacle_speed_limit = Imagespace(([speed_limit_entry speed_limit_bottom;speed_limit_entry speed_limit_top;speed_limit_exit speed_limit_bottom; speed_limit_exit speed_limit_top]'*1)');
Obstacles = Obstacles.addnode(0,obstacle_speed_limit);
% plot obstacles
figure(fig_points)
obstacle_speed_limit.P.plot('color','black','alpha',1);
figure(fig_trajectories)
obstacle_speed_limit.P.plot('color','black','alpha',1);


% algorithm parameters
N_sample_max = 100; % max number of samples

%% Fictitious points (to be removed). Used for debug purposes.
% PUNTI_FINTI = [6 1;
%     6 2;
%     6 3;
%     5 3;
%     5 2;
%     4 2;
%     5 0]';
% PUNTI_FINTI = [   13.1138    2.5570   -1.8239    7.3473   23.3957   17.0000 5 ; 
%     2.6741    2.3733   -2.8106    2.8929    0.5705         0 0];
% load prova_punti_strani.mat;
N_PUNTI_FINTI = size(PUNTI_FINTI,2);
%            PUNTI_FINTI = [6 3]';
% N_sample_max = size(PUNTI_FINTI,2);
%%
% main loop
for ii=1:N_sample_max
    %     ii
    %waitforbuttonpress
    %% sampling
    if mod(ii,10)==0
        z_rand = z_goal(1:2); % every once in a while push in a known number
            elseif ii <= N_PUNTI_FINTI
                z_rand = PUNTI_FINTI(:,ii);
    else
        z_rand = Chi0.sample; % sample a point in Chi0.
        %         x_rand = PUNTI_FINTI(:,ii); % comment this out to test the algo with random points
    end
    
    if verbose
        figure(fig_points)
        %     plot(x_rand(1),x_rand(2),'x','linewidth',2)
        figure(fig_trajectories)
        plot(z_rand(1),z_rand(2),'x','linewidth',2)
    end
    %% Run RRT* on the Chi0 space
    Chi = Chi0;
    [T,G,E] = localRRTstar(Chi,Ptree,z_rand,T,G,E,Obstacles,verbose);
    % check if has added the goal as last node
    dim = ~isnan(z_goal);
    if reached(T.Node{end},z_goal)
        idz_Goal = T.nnodes; % last one is the goal state, for the moment (in anytime version this will change).
        disp('Raggiunto!');
        %         plot(traj_pos,traj_vel,'linewidth',2,'color','yellow')
        break
    end
    %% Get last added node
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

disp('PLANNING COMPLETATO')

%% simulate the optimal plan that has been found
source_node = 1;
goal_node = idz_Goal; % just to try
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

x_values=z_init(1);
y_values=z_init(2);
for k=2:length(opt_plan.Node)
    x_values = horzcat(x_values,opt_plan.Node{k}.primitive_q(2));
    y_values = horzcat(y_values,opt_plan.Node{k}.primitive_q(4));
    figure(fig_points)
    line(x_values, y_values,'color','yellow','LineWidth',4);
    %     figure(fig_trajectories)
    %     line(x_values, y_values,'color','yellow','LineWidth',4);
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
            %             keyboard
            
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
            [time,traj_x_cart]=gen_primitives_muovi_local(primitive_muovi_params);
            traj_y_cart = zeros(size(traj_x_cart));
            q_reference_add = [q_reference(1,end)+time(:)';
                traj_x_cart(:)';
                traj_y_cart(:)'];
            q_reference = [q_reference, q_reference_add];
            % plot the trajectory on the phase plane
            figure(fig_trajectories)
            traj_pos = xi+cumtrapz(time,traj_x_cart);
            traj_vel = traj_x_cart;
            line(traj_pos, traj_vel,'color','yellow','LineWidth',4);
        otherwise
            disp('standby');
    end
end
save([run_filepath 'runna.mat'],'q_reference');
save([run_filepath 'rsim_tfdata.mat'],'q_reference');
Tend = q_reference(1,end)*1.1; % 10 percent more time, for the show
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
% and the show must go on!
load ../example/optimal.mat;
figure
plot(rt_t,rt_qp_ref,rt_t,rt_qp,q_reference(1,:),q_reference(2,:))
grid on
anima