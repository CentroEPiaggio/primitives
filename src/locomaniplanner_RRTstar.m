% locomaniplanner
clear all; clear import; close all; clc;

verbose = 1;

run utils/startup_lmp.m;

import primitive_library.*;

movie=1;

if movie==1
    frames=1;
    vidObj=VideoWriter('rrtstar');
    vidObj.Quality = 100;
    vidObj.FrameRate = 1;
    open(vidObj);
end

% actually instead of NaN we could use a value. Why is it better to use
% NaN? We'll see.
z_init = [0  ;0  ;NaN;NaN]; % initial state. Position and speed of cart are both zeros
z_goal = [NaN;NaN;NaN;  1]; % goal state. Button shall be pressed
z_goal = [17;   0;NaN;NaN]; % goal state for debug
z_goal = [17;   0;1;NaN]; % goal state for debug

[T,G,E] = InitializeTree();
[T,G,E] = InsertNode(0,z_init,T,G,E,[],0,0); % add first node
% [Ptree,Chi0]=InitializePrimitives(); % builds a list with all available primitives

InitializePrimitives; % builds Ptree, which is a list with all available primitives, and Chi0: which is the common space

fig_chi0 = 2; % figure handle to plot the sampled points and their connections (i.e. graph vertices and edges)
fig_xy = 3;
fig_yv = 4;
% fig_trajectories = 3; % figure handle to plot the sampled points and their trajectories
figure(fig_chi0)
Chi0.P.plot('color','lightgreen','alpha',0.5);hold on;     % plot search region (piano)
% Chi1.P.plot('color','lightblue','alpha',0.5)
axis equal;
plot(z_init(1),z_init(2),'go','linewidth',4) % plot initial point
plot(z_goal(1),z_goal(2),'ko','linewidth',4) % plot initial point
figure(fig_xy)
plot3(z_init(1),z_init(2),1,'go','linewidth',4) % plot initial point % HARDFIX 1 in z_init(3)
hold on
plot3(z_goal(1),z_goal(2),z_goal(3),'ko','linewidth',4) % plot initial point
% figure(fig_trajectories)
% Chi0.P.plot('color','lightgreen','alpha',0.5);hold on;     % plot search region (piano)
% axis equal;
% plot(z_init(1),z_init(2),'go','linewidth',4) % plot initial point
% plot(z_goal(1),z_goal(2),'ro','linewidth',4) % plot initial point
plot_nodes=0;
plot_edges=0;

% define obstacles
Obstacles = tree;
speed_limit_bottom = -1;
speed_limit_top = 1;
speed_limit_entry = 10;
speed_limit_exit = 15;
% speed_limit_bottom = 0;
% speed_limit_top = 0;
% speed_limit_entry = 0;
% speed_limit_exit = 0;
obstacle_speed_limit = Imagespace(([speed_limit_entry speed_limit_bottom;speed_limit_entry speed_limit_top;speed_limit_exit speed_limit_bottom; speed_limit_exit speed_limit_top]'*1)');
Obstacles = Obstacles.addnode(0,obstacle_speed_limit);
% plot obstacles
figure(fig_chi0)
obstacle_speed_limit.P.plot('color','black','alpha',1);
figure(fig_xy)
Chi1.P.plot('color','lightblue','alpha',0.5); hold on;
% figure(fig_trajectories)
% obstacle_speed_limit.P.plot('color','black','alpha',1);

% algorithm parameters
N_sample_max = 300; % max number of samples

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
load prova_punti_strani.mat;
N_PUNTI_FINTI = size(PUNTI_FINTI,2);
% N_sample_max = size(PUNTI_FINTI,2);

cost_vector = [];
N_cost_vector = [];
%% Main loop
stop=false;
path_found = false;
for ii=1:N_sample_max
    %% sampling
    if mod(ii,10)==0 && ~path_found
        z_rand = z_goal(1:2); % every once in a while push in a known number
        %elseif ii <= N_PUNTI_FINTI
        %    z_rand = PUNTI_FINTI(:,ii);
    else
        z_rand = Chi0.sample; % sample a point in Chi0.
        %         x_rand = PUNTI_FINTI(:,ii); % comment this out to test the algo with random points
    end
    
    if verbose
        figure(fig_chi0)
        plot(z_rand(1),z_rand(2),'rx','linewidth',2)
        %         figure(fig_trajectories)
        %         plot(z_rand(1),z_rand(2),'x','linewidth',2)
        if movie==1
            movie_rrtstar(frames) = getframe(figure(fig_chi0));
            frames = frames +1;
        end
    end
    
    %% Run RRT* on the Chi0 space
    Chi = Chi0;
    [T,G,E,z_new,plot_nodes,plot_edges,feasible] = localRRTstar(Chi,Ptree,1,z_rand,T,G,E,Obstacles,verbose,plot_nodes,plot_edges);
    % check if has added the goal as last node
    dim = ~isnan(z_goal);
    if reached(T.Node{end},z_goal)
        idz_Goal = T.nnodes; % last one is the goal state, for the moment (in anytime version this will change).
        disp('Goal reached (via Muovi)!');
        path_found = true;
        keyboard
        %         plot(traj_pos,traj_vel,'linewidth',2,'color','yellow')
        break
    end
    %% Check for available primitives to extend the last sampled point in a new dimension
    idx_avail_prim = CheckAvailablePrimitives(z_new,Ptree);
    
    %% Iterate over available primitives
    if feasible
        for jj=2:length(idx_avail_prim) % first element of idx_avail_prim is conventionally associated with a unique primitive on Chi0
            if ~idx_avail_prim(jj)
                % this primitive jj is not available for this point
                continue;
            end
            prim = Ptree.Node{jj};
            % Extend the z_new point (already in the tree) with its initial_extend
            % values (see PrimitiveFun.extend)
            z_new_temp=prim.extend(z_new);
            z_new = fix_nans(z_new_temp,prim.dimensions);
            
            T.Node{T.nnodes} = z_new;
            
            if mod(ii,10)==0
                z_aug = z_goal(1:3); % every once in a while push in a known number
            else
                z_aug = prim.chi.sample;
            end
            %             z_aug(Ptree.Node{1}.dimensions>0) = z_new; % how to choose the extension is a delicate thing
            %             z_aug(2) = z_new(2); % force v constant
            figure(fig_xy);
            plot3(z_aug(1),z_aug(2),z_aug(3),'rx','linewidth',2)
            Chi_aug = prim.chi;
            cprintf('red','Sto per provare con la eleva')
            [T,G,E,z_new,plot_nodes,plot_edges] = localRRTstar(Chi_aug,Ptree,jj,z_aug,T,G,E,Obstacles,verbose,plot_nodes,plot_edges);
            if reached(T.Node{end},z_goal)
                idz_Goal = T.nnodes; % last one is the goal state, for the moment (in anytime version this will change).
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
                keyboard
                stop = true;
                break
            end
        end
        if stop
            disp('Found a feasible path!');
            source_node = 1;
            goal_node = idz_Goal; % current goal node
            % visualize tree structure with optimal path in red
            [path,cost]=plot_biograph(source_node,goal_node,G);
            % save cost and iteration for plotting (anytime) stuff
            cost_vector = [cost_vector, cost];
            N_cost_vector = [N_cost_vector, ii];
            figure(10)
            bar(N_cost_vector,cost_vector); xlabel('Iterations'); ylabel('cost');
            keyboard
            stop=false;
%             break;
        end
    end
end

disp('PLANNING COMPLETED')

%% simulate the optimal plan that has been found

%% obtain plan as the shortest path in the graph
opt_plan=extract_plan(T,E,path);

%% TODO: FIX MOVIE STUFF
% figure(fig_chi0)
% line(x_values, y_values,'color','yellow','LineWidth',4);
% %     figure(fig_trajectories)
% %     line(x_values, y_values,'color','yellow','LineWidth',4);
% if(movie==1)
%     movie_rrtstar(frames) = getframe(figure(fig_chi0));
%     frames = frames +1;
% end

save_test_data
%% showtime!
movie = 1;
%% go to test the plan
run_plan