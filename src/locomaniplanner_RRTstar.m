% locomaniplanner
clear all; clear import; close all; clc;
push_bias_freq = 9;

multiple_primitives = 1; % testing locomotion primitive only for coffee
obstacles_on = true;

% Algorithm's parameters
gam = 1000; % constant for radius of search of near nodes in near.m
tol=0.1;0.05; % tolerance for the goal region distance from the goal point

% debug and visualization flags
debug=0; % enable breakpoints
verbose = 1; % to plot stuff
movie=1; % to save video of stuff. Movie has been modified into a
% post-processing option. See run_plan.m and anima.m.

% load libraries
load_libraries

% actually instead of NaN we could use a value. Why is it better to use
% NaN? We'll see.
z_init = [0  ; 0 ; 0 ; 0 ; NaN]; % initial state: [x,y,theta,v, tau].
z_goal = [9  ; 9 ; 0 ; 0 ;   1]; % goal state:    [position,speed,end-effector height, object grasped].
z_goal = [1.5  ; 0.5 ; 0 ; 0 ;   1]; % goal state:    [position,speed,end-effector height, object grasped].
L_arm = 0.31;
z_init = [0  ; 0 ; 0 ; 0 ; NaN]; % initial state: [x,y,theta,v, tau].
z_goal = [L_arm  ; L_arm ; pi/4 ; 0 ;   1]; % goal state:    [position,speed,end-effector height, object grasped].
z_goal = [0  ; 1 ; pi ; 0 ;   1]; % goal state:    [position,speed,end-effector height, object grasped].
%%
[T,G,E] = InitializeTree();
[~,T,G,E] = InsertNode(0,z_init,T,G,E,[],0,0); % add first node

InitializePrimitives; % builds Ptree, which is a list with all available primitives, and Chi0: which is the common space

InitObstacles; % initialize obstacles structure

% These points are added to bias the sampling towards points we want the
% solution to pass by.
%z_intermediate_1 = [0;0;0;];
% z_intermediate_2 = [15;0;1];
% z_intermediate_1 = [mean([xmin_grasping,xmax_grasping]); mean([ymin_grasping,ymax_grasping]); 0 ; 0 ; 0 ]; % TODO: how to bias only on [x,y,tau] for any value of [v,w]?
if multiple_primitives
    z_intermediate_1 = [1; 1; pi/2 ; 0 ; 0 ]; % TODO: how to bias only on [x,y,tau] for any value of [v,w]?
    z_intermediate_2 = [mean([xmin_grasping,xmax_grasping]); mean([ymin_grasping,ymax_grasping]); pi/2 ; 0 ; 1 ]; % TODO: how to bias only on [x,y,tau] for any value of [v,w]?
    z_intermediate_3 = [1; 0.25; pi/2; 0; 0];
    z_intermediate_4 = [1; 0.55; pi/2; 0; 1];
    bias_points_generic = {z_intermediate_1,z_intermediate_2,z_intermediate_3,z_intermediate_4,z_goal};
else
    bias_points_generic = {z_goal};
end
bias_points = bias_points_generic;
% bias_points = {z_goal, z_intermediate_2};
% bias_points = {z_goal, z_intermediate_1, z_intermediate_2};
bias_ii = 1;

InitView; % open figures

% algorithm parameters
N_sample_max = 1000; % max number of samples

% These vectors are used to save progress of anytime algorithm
cost_vector = [];
N_cost_vector = [];

%% Main loop
stop=false;
path_found = false;
goal_node = [];
source_node = 1;
opt_path_edges = {};
replicate_over_primitive = []; % list of primitives that need replicated points
global raggio_conta; raggio_conta=1; figure(13); plot(0,0);hold on;
z_rand = Chi0.sample; pushed_in_goal=0;
z_aug = Ptree.Node{2}.chi.sample;

bag_bias = {};
bias_points = [bias_points_generic];
%return
for ii=1:N_sample_max
    cprintf('*[1,0.5,0]*','# %d\n',ii);
    cprintf('*[0,0.7,1]*','* sampling z_rand *\n');
    %% sampling
    if mod(ii,push_bias_freq)==0 %&& ~path_found
        z_bias = bias_points{bias_ii}; bias_ii = bias_ii+1; if bias_ii>length(bias_points), bias_ii=1; end
        z_rand = z_bias(Ptree.Node{1}.dimensions>0); % every once in a while push in a known number
        disp('Pushing in goal')
        pushed_in_goal=1;
    elseif ~isempty(bag_bias)
        z_rand = bag_bias{1};
        bag_bias = {bag_bias{2:end}};
    else
        z_rand = Chi0.sample; % sample a point in Chi0.
        pushed_in_goal=0;
        z_rand1 = z_rand; z_rand1(3) = 0;
        z_rand2 = z_rand; z_rand2(3) = pi/4;
        z_rand3 = z_rand; z_rand3(3) = pi/2;
        bag_bias = {z_rand1, z_rand2, z_rand3};
    end
    while ~CollisionFree(fix_nans(z_rand,Ptree.Node{1}.dimensions_imagespace),Ptree,Obstacles)
        if mod(ii,push_bias_freq)==0 %&& ~path_found
            z_bias = bias_points{bias_ii}; bias_ii = bias_ii+1; if bias_ii>length(bias_points), bias_ii=1; end
            z_rand = z_bias(Ptree.Node{1}.dimensions>0); % every once in a while push in a known number
            disp('Pushing in goal')
            pushed_in_goal=1;
        else
            z_rand = Chi0.sample; % sample a point in Chi0.
            pushed_in_goal=0;
        end
    end
    
    % round up to the second decimal
    z_rand = round(z_rand*100)/100;
        
    if verbose
        disp(['z_rand: ' num2str(z_rand(:)')])
        fig_xv=2; fig_xy = 3; % parametrize
        figure(fig_chi0)
        plot(z_rand(1),z_rand(2),'rx','linewidth',2)
        figure(fig_xy)
        plot3(z_rand(1),z_rand(2),0,'rx','linewidth',2) % TODO: this has to be parametrized
    end
    
    %% Run sampling algorithm on the Chi0 space
    idx_parent_primitive = [];
    [T,G,E,z_new,plot_nodes,plot_edges,feasible,added_new,idx_last_added] = localRRTstar(Chi0,Ptree,1,z_rand,T,G,E,Obstacles,verbose,plot_nodes,plot_edges,pushed_in_goal,goal_node,idx_parent_primitive,gam,tol,replicate_over_primitive);
    %     test_plot_opt
    if added_new && reached(T.Node{end},z_goal,tol) % first time a path is found
        %-keyboard
        idz_Goal = T.nnodes; % last one is the goal state, for the moment (in anytime version this will change).
        goal_node = idz_Goal;
        disp('Goal reached (via Muovi)!');
        path_found = true;
        if debug
            %-keyboard
        end
        
        load handel;
        player = audioplayer(y, Fs);
        play(player);
        [cost,opt_path,~] = graphshortestpath(G,source_node,goal_node);
        %         %-keyboard
        opt_path_edges = plot_opt_path(T,opt_path,opt_path_edges);
        % save cost and iteration for plotting (anytime) stuff
        if  isempty(cost_vector) || (~isempty(cost_vector) && cost<cost_vector(end))
            cost_vector = [cost_vector, cost];
            N_cost_vector = [N_cost_vector, ii];
            plot_biograph(source_node,goal_node,G);
            figure(10)
            bar(N_cost_vector,cost_vector); xlabel('Iterations'); ylabel('cost');
            %             hold on
            save_test_data
            if debug
                %-keyboard
            end
        end
        continue % for anytime behavior
    end
    % check if path has been found and it needs to plot and save data
    if path_found % anytime optimization. If one feasible path was
        % found in a previous iteration, keep optimizing
        [cost,opt_path,~] = graphshortestpath(G,source_node,goal_node);
        opt_path_edges = plot_opt_path(T,opt_path,opt_path_edges);
        % save cost and iteration for plotting (anytime) stuff
        if  isempty(cost_vector) || (~isempty(cost_vector) && cost<cost_vector(end))
            cost_vector = [cost_vector, cost];
            N_cost_vector = [N_cost_vector, ii];
            plot_biograph(source_node,goal_node,G);
            figure(10)
            bar(N_cost_vector,cost_vector); xlabel('Iterations'); ylabel('cost');
            %             hold on
            save_test_data
            if debug
                %-keyboard
            end
        end
    end
    
    if feasible && added_new % last call to localRRTstart has produced a new
        % node z_new which was added to the tree?
        % Check for available primitives to extend the last sampled point in a new dimension
        cprintf('*[0,0.7,1]*','* Check for available primitives to extend the last point in new dimensions *\n');
        idx_avail_prim = CheckAvailablePrimitives(z_new,Ptree);
        
        %% Iterate over available primitives
        for jj=2:length(idx_avail_prim) % first element of idx_avail_prim is conventionally associated with a unique primitive on Chi0
            cprintf('*[0,.7,0.5]*','* Iterate over all available primitives *\n');
            if ~idx_avail_prim(jj)
                % this primitive jj is not available for this point. Keep going.
                continue;
            end
            
            prim = Ptree.Node{jj};
            
            %% take the last added point, extend it, and add it to the tree
            %-keyboard
            
            %% TRIGGER ON
            q_trig = [];
            Ts = 0.01; % TODO: once and for all parametrize this
            cost_trig = Ts;
            time_trig = 0:Ts:Ts;
            x_trig = [T.Node{idx_last_added} prim.extend(z_new)];
            [added_new,T,G,E,... % inputs for algorithm stuff
                plot_nodes,plot_edges, ... % inputs for plotting visual stuff
                idx_last_added] ... % return index of last added node
                = InsertExtendedNode(idx_last_added,prim.extend(z_new),T,G,E, prim, q_trig, cost_trig, x_trig , time_trig, verbose, plot_nodes, plot_edges);
            prim.extend(z_new)
            if checkdiscontinuity(T,E,Ptree)
                %-keyboard
            end
            
            
            %%
            if added_new && reached(T.Node{end},z_goal,tol)
                %-keyboard
                idz_Goal = T.nnodes; % last one is the goal state, for the moment (in anytime version this will change).
                goal_node = idz_Goal;
                % the -1 is a dirty fix for the fact
                % that this node is inserted two
                % times, once via prim.extend and
                % one from localRRT*(Chi_aug). Does
                % not happen always but still it
                % needs this workaround for those
                % times.
                disp('Goal reached (via Eleva)!');
                path_found = true;
                %         plot(traj_pos,traj_vel,'linewidth',2,'color','yellow')
                if debug
                    %-keyboard
                end
                stop = true;
                break
            end
            if path_found
                %         [path,cost]=plot_biograph(source_node,goal_node,G);
                [cost,opt_path,~] = graphshortestpath(G,source_node,goal_node);
                opt_path_edges = plot_opt_path(T,opt_path,opt_path_edges);
                % save cost and iteration for plotting (anytime) stuff
                if ~isempty(cost_vector) && cost<cost_vector(end)
                    cost_vector = [cost_vector, cost];
                    N_cost_vector = [N_cost_vector, ii];
                    plot_biograph(source_node,goal_node,G);
                    figure(10)
                    bar(N_cost_vector,cost_vector); xlabel('Iterations'); ylabel('cost');
                    %                     hold on
                    save_test_data
                    if debug
                        %-keyboard
                    end
                end
            end
            % RESTART FROM HERE: see if it's better to have z_aug only as
            % the extra-dimensions, to augment it with the point on the
            % base space, or a mix of the two.
            % for the moment being, we just keep it as a sample in the
            % primitive's image space, i.e., the size of the sample is
            % given by the dimension of the image space
            if mod(ii,push_bias_freq)==0 %&& ~path_found
                z_aug = z_bias(prim.dimensions_imagespace>0); % every once in a while push in a known number
                disp('Pushing in aug goal')
                pushed_in_goal=1;
            else
                z_aug = prim.chi.sample;
                %                 z_aug(Ptree.Node{1}.dimensions>0) = z_rand; % BUGFIX DISCONT: only sample in the third dimension, starting from the already added point
                pushed_in_goal=0;
            end
            while ~CollisionFree(fix_nans([z_aug],Ptree.Node{2}.dimensions_imagespace),Ptree,Obstacles)
                if mod(ii,push_bias_freq)==0 %&& ~path_found
                    z_aug = z_bias(prim.dimensions_imagespace>0); % every once in a while push in a known number
                    disp('Pushing in aug goal')
                    pushed_in_goal=1;
                else
                    z_aug = prim.chi.sample;
                    %                 z_aug(Ptree.Node{1}.dimensions>0) = z_rand; % BUGFIX DISCONT: only sample in the third dimension, starting from the already added point
                    pushed_in_goal=0;
                end
            end
            % rounding up to the second decimal
            z_aug = round(z_aug*100)/100;
            
            Chi_aug = prim.chi;
            
            if verbose
                disp(['z_aug: ' num2str(z_aug(:)')])
                figure(fig_chi0)
                plot(z_rand(1),z_rand(2),'rx','linewidth',2)
                figure(fig_xy);
                plot3(z_aug(1),z_aug(2),z_aug(3),'rx','linewidth',2)
                cprintf('red','Sto per provare con la %s\n',prim.name)
            end
            
            idx_parent_primitive = 1;
            
            [T,G,E,z_new_aug,plot_nodes,plot_edges,feasible,added_new,idx_last_added] = localRRTstar(Chi_aug,Ptree,jj,z_aug,T,G,E,Obstacles,verbose,plot_nodes,plot_edges,pushed_in_goal,goal_node,idx_parent_primitive,gam,tol,replicate_over_primitive);
            %% set of points with different dimensions along the primitives
            if added_new
                %-keyboard
                z_added = T.get(idx_last_added);
                %                 z_new(5)==1 % prim.istask % boolean
                if z_added(5)==1 % TODO: HARDCODED % verify condition on trigger exit
                    %-keyboard
                    %                     replicate_over_primitive = [replicate_over_primitive, prim.ID];
                    replicate_over_primitive = 2;
                end
            end
            
            if added_new && reached(T.Node{end},z_goal,tol)
                idz_Goal = T.nnodes; % last one is the goal state, for the moment (in anytime version this will change).
                goal_node = idz_Goal;
                % the -1 is a dirty fix for the fact
                % that this node is inserted two
                % times, once via prim.extend and
                % one from localRRT*(Chi_aug). Does
                % not happen always but still it
                % needs this workaround for those
                % times.
                disp('Goal reached (via Eleva)!');
                path_found = true;
                %         plot(traj_pos,traj_vel,'linewidth',2,'color','yellow')
                if debug
                    %-keyboard
                end
                stop = true;
                break
            end
            if path_found
                [cost,opt_path,~] = graphshortestpath(G,source_node,goal_node);
                opt_path_edges = plot_opt_path(T,opt_path,opt_path_edges);
                % save cost and iteration for plotting (anytime) stuff
                if ~isempty(cost_vector) && cost<cost_vector(end)
                    cost_vector = [cost_vector, cost];
                    N_cost_vector = [N_cost_vector, ii];
                    plot_biograph(source_node,goal_node,G);
                    figure(10)
                    bar(N_cost_vector,cost_vector); xlabel('Iterations'); ylabel('cost');
                    %                     hold on
                    save_test_data
                    if debug
                        %-keyboard
                    end
                end
            end
            
        end
        
        if stop
            disp('Found a feasible path!');
            source_node = 1;
            goal_node = idz_Goal; % current goal node
            % visualize tree structure with optimal path in red
            [opt_path,cost]=plot_biograph(source_node,goal_node,G);
            % save cost and iteration for plotting (anytime) stuff
            cost_vector = [cost_vector, cost];
            N_cost_vector = [N_cost_vector, ii];
            figure(10)
            bar(N_cost_vector,cost_vector); xlabel('Iterations'); ylabel('cost');
            %             hold on
            save_test_data
            if debug
                %-keyboard
            end
            stop=false;
            %             break;
        end
    end
end

disp('PLANNING COMPLETED')

%% simulate the optimal plan that has been found

%% obtain plan as the shortest path in the graph
opt_plan=extract_plan(T,E,opt_path);

%%
save_test_data
%% showtime!
movie = 1;
%% go to test the plan
run_plan
