% locomaniplanner
clear all; clear import; close all; clc;
push_goal_freq = 10;

multiple_primitives = 0;

% debug and visualization flags
debug=0; % enable breakpoints
verbose = 1; % to plot stuff
movie=1; % to save video of stuff. Movie has been modified into a
% post-processing option. See run_plan.m and anima.m.

% load libraries
run utils/startup_lmp.m;
import primitive_library.*;

% actually instead of NaN we could use a value. Why is it better to use
% NaN? We'll see.
z_init = [0  ;0  ;1;NaN]; % initial state: [position,speed,end-effector height].
z_init = [0  ;0  ;NaN;NaN]; % initial state: [position,speed,end-effector height].
% z_goal = [20;   0;3;NaN]; % goal state:    [position,speed,end-effector height].
z_goal = [20;   0;1;NaN]; % goal state:    [position,speed,end-effector height].
z_goal = [20;   0;NaN;NaN]; % goal state:    [position,speed,end-effector height].

[T,G,E] = InitializeTree();
[~,T,G,E] = InsertNode(0,z_init,T,G,E,[],0,0); % add first node

InitializePrimitives; % builds Ptree, which is a list with all available primitives, and Chi0: which is the common space

InitObstacles; % initialize obstacles structure

InitView; % open figures

% algorithm parameters
N_sample_max = 3000; % max number of samples

% These vectors are used to save progress of anytime algorithm
cost_vector = [];
N_cost_vector = [];

%% Main loop
stop=false;
path_found = false;
goal_node = [];
source_node = 1;
opt_path_edges = {};

for ii=1:N_sample_max
    cprintf('*[1,0.5,0]*','# %d\n',ii);
    cprintf('*[0,0.7,1]*','* sampling z_rand *\n');
    %% sampling
    if mod(ii,push_goal_freq)==0 %&& ~path_found
        z_rand = z_goal(1:2); % every once in a while push in a known number
        disp('Pushing in goal')
        pushed_in_goal=1;
    else
        z_rand = Chi0.sample; % sample a point in Chi0.
        pushed_in_goal=0;
    end
    
    % round up to the second decimal
    z_rand = round(z_rand*100)/100;
    
    if verbose
        disp(['z_rand: ' num2str(z_rand(:)')])
        figure(fig_chi0)
        plot(z_rand(1),z_rand(2),'rx','linewidth',2)
    end
    
    %% Run sampling algorithm on the Chi0 space
    [T,G,E,z_new,plot_nodes,plot_edges,feasible,added_new] = localRRTstar(Chi0,Ptree,1,z_rand,T,G,E,Obstacles,verbose,plot_nodes,plot_edges,pushed_in_goal,goal_node);
    test_plot_opt
    if reached(T.Node{end},z_goal) % first time a path is found
%         keyboard
        idz_Goal = T.nnodes; % last one is the goal state, for the moment (in anytime version this will change).
        goal_node = idz_Goal;
        disp('Goal reached (via Muovi)!');
        path_found = true;
        if debug,keyboard,end
        
        load handel;
        player = audioplayer(y, Fs);
        play(player);
        [cost,opt_path,~] = graphshortestpath(G,source_node,goal_node);
        keyboard
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
            if debug,keyboard,end
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
            if debug,keyboard,end
        end
    end
    
    %     % check if path has been found and it needs to plot and save data
    %     if path_found % anytime optimization. If one feasible path was
    %                   % found in a previous iteration, keep optimizing
    %         [cost,opt_path,~] = graphshortestpath(G,source_node,goal_node);
    %         % save cost and iteration for plotting (anytime) stuff
    %         if ~isempty(cost_vector) && cost<cost_vector(end)
    %             cost_vector = [cost_vector, cost];
    %             N_cost_vector = [N_cost_vector, ii];
    %             plot_biograph(source_node,goal_node,G);
    %             figure(10)
    %             bar(N_cost_vector,cost_vector); xlabel('Iterations'); ylabel('cost');
    %             %             hold on
    %             save_test_data
    %             if debug,keyboard,end
    %         end
    % %     elseif pushed_in_goal || reached(T.Node{end},z_goal) % first time a path is found
    %     elseif reached(T.Node{end},z_goal) % first time a path is found
    %         keyboard
    %         idz_Goal = T.nnodes; % last one is the goal state, for the moment (in anytime version this will change).
    %         goal_node = idz_Goal;
    %         disp('Goal reached (via Muovi)!');
    %         path_found = true;
    %         if debug,keyboard,end
    %
    %         load handel;
    %         player = audioplayer(y, Fs);
    %         play(player);
    %
    %         continue % for anytime behavior
    %     end
    
    
    if feasible && added_new % last call to localRRTstart has produced a new
        % node z_new which was added to the tree?
        % Check for available primitives to extend the last sampled point in a new dimension
        cprintf('*[0,0.7,1]*','* Check for available primitives to extend the last point in new dimensions *\n');
        idx_avail_prim = CheckAvailablePrimitives(z_new,Ptree);
        
        % Iterate over available primitives
        for jj=2:length(idx_avail_prim) % first element of idx_avail_prim is conventionally associated with a unique primitive on Chi0
            cprintf('*[0,.7,0.5]*','* Iterate over all available primitives *\n');
            if ~idx_avail_prim(jj)
                % this primitive jj is not available for this point. Keep going.
                continue;
            end
            
            prim = Ptree.Node{jj};
            
            % Extend the z_new point (already in the tree) with its initial_extend
            % values (see PrimitiveFun.extend)
            z_whatwas = z_new
            z_new_temp=prim.extend(z_new)
            z_new_extended = fix_nans(z_new_temp,prim.dimensions)
            
            T.Node{T.nnodes} = z_new_extended;
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
                    if debug,keyboard,end
                end
            elseif pushed_in_goal || reached(T.Node{end},z_goal)
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
                if debug,keyboard,end
                stop = true;
                break
            end
            
            if mod(ii,push_goal_freq)==0 %&& ~path_found
                z_aug = z_goal(1:3); % every once in a while push in a known number
                disp('Pushing in aug goal')
                pushed_in_goal=1;
            else
                z_aug = prim.chi.sample;
                pushed_in_goal=0;
            end
            
            % rounding up to the second decimal
            z_aug = round(z_aug*100)/100;
            
            Chi_aug = prim.chi;
            
            if verbose
                disp(['z_aug: ' num2str(z_aug(:)')])
                figure(fig_xy);
                plot3(z_aug(1),z_aug(2),z_aug(3),'rx','linewidth',2)
                cprintf('red','Sto per provare con la eleva')
            end
            
            [T,G,E,z_new_aug,plot_nodes,plot_edges] = localRRTstar(Chi_aug,Ptree,jj,z_aug,T,G,E,Obstacles,verbose,plot_nodes,plot_edges,pushed_in_goal,goal_node);
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
                    if debug,keyboard,end
                end
            elseif pushed_in_goal || reached(T.Node{end},z_goal)
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
                if debug,keyboard,end
                stop = true;
                break
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
            if debug,keyboard,end
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
