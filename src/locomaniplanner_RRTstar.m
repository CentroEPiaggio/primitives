% locomaniplanner
clear all; clear import; close all; clc;
push_bias_freq = 5;
warm_start = 0;

using_yarp = 0;
multiple_primitives = 1; % testing locomotion primitive only for coffee
obstacles_on = true;

% Algorithm's parameters
gam = 1000;%1000; % constant for radius of search of near nodes in near.m
tol=0.1;0.05; % tolerance for the goal region distance from the goal point

% debug and visualization flags
debug=0; % enable breakpoints
verbose = 1; % to plot stuff
movie=1; % to save video of stuff. Movie has been modified into a
% post-processing option. See run_plan.m and anima.m.

% load libraries
load_libraries

L_arm = 0.5;
z_init = [0  ; 0 ; 0 ; 0 ; NaN]; % initial state: [x,y,theta,v, tau].
idx_z_init_image_space = 1; % Specify the image space where the starting sample lives
z_goal = [2  ; 0 ; 0 ; 0 ;   1]; % goal state:    [position,speed,end-effector height, object grasped].
%%
% initializing yarp

if using_yarp
    LoadYarp;
    
    read_port=yarp.Port;
    read_port.close;
    read_port.open('/locomanipulation/command:i');
    
    recv=yarp.Bottle;
    
    write_port=yarp.Port;
    write_port.close;
    write_port.open('/locomanipulation_solution/command:o');
    
    send=yarp.Bottle;
    
    pause(1);
end
%%
% receving data from yarp, start this after opening the write port
if using_yarp
    
    if(~read_port.isOpen)
        read_port.open('/locomanipulation/command:i');
    end
    
    recv.fromString('');
    while(strcmp(recv.toString, ''))
        read_port.read(recv);
        pause(0.01);
    end
    
    list = recv.get(0).asList();
    
    command_obj = list.get(0).asString();
    
    frame_obj = list.get(1).asString();
    
    x_obj = list.get(2).asDouble();
    y_obj = list.get(3).asDouble();
    z_obj = list.get(4).asDouble();
    qx_obj = list.get(5).asDouble();
    qy_obj = list.get(6).asDouble();
    qz_obj = list.get(7).asDouble();
    qw_obj = list.get(8).asDouble();
    
    sx_obj = list.get(9).asDouble();
    sy_obj = list.get(10).asDouble();
    sz_obj = list.get(11).asDouble();
    
    disp([char(command_obj), ': ', char(frame_obj), ' ', num2str(x_obj), ' ', num2str(y_obj), ' ', num2str(z_obj), ' ', num2str(qx_obj), ' ', num2str(qy_obj), ' ', num2str(qz_obj), ' ', num2str(qw_obj), ' ', num2str(sx_obj), ' ', num2str(sy_obj), ' ', num2str(sz_obj),])
    
    read_port.close;
end
%% Initialization scripts
[T,G,E] = InitializeTree();
[~,T,G,E] = InsertNode(0,z_init,T,G,E,[],0,0); % add first node

if warm_start
    load warm_start_data.mat;
end

InitializePrimitives; % builds Ptree, which is a list with all available primitives, and Chi0: which is the common space

% active_primitives = zeros(1,Ptree.nnodes); % initialize the activated primitives with the first one (locomotion)
% the variable active_primitives has boolean values, where the {i}-th element true means that
% the primitive with the ID {i} is active
active_primitives = CheckAvailablePrimitives(z_init,Ptree,idx_z_init_image_space); % initialize the activated primitives

InitObstacles; % initialize obstacles structure

% These points are added to bias the sampling towards points we want the
% solution to pass by.
z_bias = [];
if multiple_primitives
    z_intermediate_1 = [x_target-0.1; y_target; 0; 0; 0];
    z_intermediate_2 = [x_target; y_target; 0; 0; 1];
    z_intermediate_3 = [x_target-0.2; y_target-0.3; 0; 0; 0];
    z_intermediate_4 = [x_target-0.2; y_target-0.3; 0; 0; 1];
    bias_points_generic = {z_intermediate_1,z_intermediate_2,z_intermediate_3,z_intermediate_4,z_goal};
    bias_points_image_space = {1, 2, 2, 2, 1};
else
    z_goal(end) = NaN;
    bias_points_generic = {[1,0.4,0,0,NaN]',z_goal,[0.5,0,0,0,NaN]',[0.75,0,0,0,NaN]',[0.8,0,0,0,NaN]',[0.85,0,0,0,NaN]',[1,0,0,0,NaN]',[1.2,0,0,0,NaN]',[1.5,0,0,0,NaN]',[1.6,0,0,0,NaN]',[1.8,0,0,0,NaN]',z_goal};
    bias_points_image_space = {1,1,1,1,1,1,1,1,1,1,1,1};
end
bias_points = bias_points_generic;
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
pushed_in_goal=0;
% z_rand = Chi0.sample;
% z_aug = Ptree.Node{2}.chi.sample;

bag_bias = {};
bias_points = [bias_points_generic];
%return
for ii=1:N_sample_max
    cprintf('*[1,0.5,0]*','# %d\n',ii);
    cprintf('*[0,0.7,1]*','* sampling z_rand *\n');
    cprintf('*[0,0.7,1]*','* Choosing the sampling image space: ');
    if mod(ii,push_bias_freq)==0 && ~isempty(bias_points) % if we want to push in a goal state in a certain primitive space, we have to force sampling in that primitive space
        sample_image_space = bias_points_image_space{bias_ii}; % DD_move primitive space
    else
        sample_image_space = RandSelectActivePrimitive(active_primitives); % uniformly randomly selects the primitive from where to sample from
    end
    cprintf('*[0,0.7,1]*','%d: %s*\n',sample_image_space,Ptree.Node{sample_image_space}.name);
    %     keyboard
    Chi_ii = Ptree.Node{sample_image_space}.chi; % select current image space where to sample from
    %% sampling
    if mod(ii,push_bias_freq)==0 && ~isempty(bias_points)%&& ~path_found
        z_bias = bias_points{bias_ii};
        z_rand = z_bias(Ptree.Node{sample_image_space}.dimensions_imagespace>0); % every once in a while push in a known number
        disp('Pushing in bias')
        if bias_ii == length(bias_points)
            disp('Pushing in goal')
            pushed_in_goal=1;
            if ~isequal(z_rand(1),z_bias(1))
                disp('Error in assigning z_rand!')
                keyboard
            end
        else
            pushed_in_goal=0;
        end
        bias_ii = bias_ii+1; if bias_ii>length(bias_points), bias_ii=1; end
    else
        z_rand = Chi_ii.sample; % sample a point in Chi_i.
        pushed_in_goal=0;
    end
    %     if active_primitives(2)
    %         keyboard
    %     end
    try
        while ~CollisionFree(fix_nans(z_rand,Ptree.Node{sample_image_space}.dimensions_imagespace),Ptree,Obstacles)
            % the next if lines should not be needed, and the change the
            % bias counter generating strange behaviors
            %             if mod(ii,push_bias_freq)==0 %&& ~path_found
            %                 z_bias = bias_points{bias_ii}; bias_ii = bias_ii+1; if bias_ii>length(bias_points), bias_ii=1; end
            %                 z_rand = z_bias(Ptree.Node{sample_image_space}.dimensions>0); % every once in a while push in a known number
            %                 disp('Pushing in goal')
            %                 pushed_in_goal=1;
            %             else
            z_rand = Chi_ii.sample; % sample a point in Chi_i.
            pushed_in_goal=0;
            %             end
        end
    catch COLLISIONMESSAGE
        disp(COLLISIONMESSAGE.message);
        keyboard
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
        %         keyboard
    end
    
    % Run steering function algorithm on the given image space space
    if sample_image_space == 1
        idx_parent_primitive = [];
    else
        idx_parent_primitive = 1; % TODO: automatize this with the primitive tree
        %         keyboard
    end
    [T,G,E,z_new,plot_nodes,plot_edges,feasible,added_new,idx_last_added] = localRRTstar(Chi_ii,Ptree,Ptree.Node{sample_image_space}.ID,z_rand,T,G,E,Obstacles,verbose,plot_nodes,plot_edges,pushed_in_goal,goal_node,idx_parent_primitive,gam,tol,replicate_over_primitive);
    %     test_plot_opt
    if multiple_primitives
        reached_goal = reached(T.Node{end},z_goal,tol,[]);
    else
        reached_goal = reached(T.Node{end},z_goal,tol,[1,2]); % for debug, just check on the x-y dimensions
    end
    if added_new && reached_goal % first time a path is found
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
    
    %% internal loop
    if feasible && added_new % last call to localRRTstart has produced a new
        % node z_new which was added to the tree?
        % Check for available primitives to extend the last sampled point in a new dimension
        cprintf('*[0,0.7,1]*','* Check for available primitives to extend the last point in new dimensions *\n');
        try
            idx_avail_prim = CheckAvailablePrimitives(z_new,Ptree,sample_image_space);
        catch MCHECK
            disp(MCHECK.message);
            keyboard
        end
        
        cprintf('*[0,0.7,1]*','Found new primitive?')
        if ~isequal(idx_avail_prim,active_primitives)
            cprintf('*[0,0.7,1]*','Found new primitive!')
            %             keyboard
            % extend the sampled node in the new image spaces
            for idx_newly_activated=1:length(active_primitives)
                if active_primitives(idx_newly_activated)==0 && idx_avail_prim(idx_newly_activated)==1 % if a primitive is not active but can be activated
                    prim = Ptree.Node{idx_newly_activated};
                    %% TRIGGER ON
                    q_trig = [];
                    Ts = 0.01; % TODO: once and for all parametrize this
                    cost_trig = Ts;
                    time_trig = 0:Ts:Ts;
                    try
                        x_trig = [T.Node{idx_last_added} prim.extend(z_new)];
                    catch MTRIG
                        disp(MTRIG.message);
                        keyboard
                    end
                    [added_new,T,G,E,... % inputs for algorithm stuff
                        plot_nodes,plot_edges, ... % inputs for plotting visual stuff
                        idx_last_added] ... % return index of last added node
                        = InsertExtendedNode(idx_last_added,prim.extend(z_new),T,G,E, prim, q_trig, cost_trig, x_trig , time_trig, verbose, plot_nodes, plot_edges);
                    prim.extend(z_new)
                    replicate_over_primitive = 2; % HARDCODED for testing, easy to generalize
                    if checkdiscontinuity(T,E,Ptree)
                        keyboard
                    end
                    if CheckForDuplicates(T,G,E)
                        disp('Ugly gesture')
                        keyboard
                    end
                end
            end
            
            active_primitives(idx_avail_prim>0) = 1; % activate new primitives
            %             keyboard
        else
            cprintf('*[0,0.7,1]*','No :(')
        end
        
        % bias removal for debug
        if ~isempty(z_bias)
            temp_T = T;
            temp_T = temp_T.addnode(temp_T.nnodes,z_bias);
            if CheckForDuplicates(temp_T,G,E)
                disp('removing bias from bias_list');
                idx_bias_to_remove = bias_ii-1;
                if idx_bias_to_remove==0 && ~isempty(bias_points)
                    idx_bias_to_remove = 1;
                end
                if idx_bias_to_remove~=0
                    bias_points_generic(idx_bias_to_remove) = [];
                    bias_points_image_space(idx_bias_to_remove) = [];
                    bias_points = [bias_points_generic];
                    if bias_ii > length(bias_points)
                        bias_ii = 1;
                    end
                end
            end
        end
        
        if CheckForDuplicates(T,G,E)
            disp('Ugly gesture')
            keyboard
        end
        
        if ~graphisdag(G)
            disp('Very ugly gesture')
            keyboard
        end
        %% INTERNAL LOOP STARTS HERE
        % Iterate over available primitives
        %         for jj=2:length(idx_avail_prim) % first element of idx_avail_prim is conventionally associated with a unique primitive on Chi0
        %             cprintf('*[0,.7,0.5]*','* Iterate over all available primitives *\n');
        %             if ~idx_avail_prim(jj)
        %                 % this primitive jj is not available for this point. Keep going.
        %                 continue;
        %             end
        %
        %             prim = Ptree.Node{jj};
        %
        %             %% take the last added point, extend it, and add it to the tree
        %             %-keyboard
        %
        %             %% TRIGGER ON
        %             q_trig = [];
        %             Ts = 0.01; % TODO: once and for all parametrize this
        %             cost_trig = Ts;
        %             time_trig = 0:Ts:Ts;
        %             try
        %                 x_trig = [T.Node{idx_last_added} prim.extend(z_new)];
        %             catch MTRIG
        %                 disp(MTRIG.message);
        %                 keyboard
        %             end
        %                     [added_new,T,G,E,... % inputs for algorithm stuff
        %                         plot_nodes,plot_edges, ... % inputs for plotting visual stuff
        %                         idx_last_added] ... % return index of last added node
        %                         = InsertExtendedNode(idx_last_added,prim.extend(z_new),T,G,E, prim, q_trig, cost_trig, x_trig , time_trig, verbose, plot_nodes, plot_edges);
        %                     prim.extend(z_new)
        %             if checkdiscontinuity(T,E,Ptree)
        %                 %-keyboard
        %             end
        %
        %
        %             %%
        %             if added_new && reached(T.Node{end},z_goal,tol)
        %                 %-keyboard
        %                 idz_Goal = T.nnodes; % last one is the goal state, for the moment (in anytime version this will change).
        %                 goal_node = idz_Goal;
        %                 % the -1 is a dirty fix for the fact
        %                 % that this node is inserted two
        %                 % times, once via prim.extend and
        %                 % one from localRRT*(Chi_aug). Does
        %                 % not happen always but still it
        %                 % needs this workaround for those
        %                 % times.
        %                 disp('Goal reached (via Eleva)!');
        %                 path_found = true;
        %                 %         plot(traj_pos,traj_vel,'linewidth',2,'color','yellow')
        %                 if debug
        %                     %-keyboard
        %                 end
        %                 stop = true;
        %                 break
        %             end
        %             if path_found
        %                 %         [path,cost]=plot_biograph(source_node,goal_node,G);
        %                 [cost,opt_path,~] = graphshortestpath(G,source_node,goal_node);
        %                 opt_path_edges = plot_opt_path(T,opt_path,opt_path_edges);
        %                 % save cost and iteration for plotting (anytime) stuff
        %                 if ~isempty(cost_vector) && cost<cost_vector(end)
        %                     cost_vector = [cost_vector, cost];
        %                     N_cost_vector = [N_cost_vector, ii];
        %                     plot_biograph(source_node,goal_node,G);
        %                     figure(10)
        %                     bar(N_cost_vector,cost_vector); xlabel('Iterations'); ylabel('cost');
        %                     %                     hold on
        %                     save_test_data
        %                     if debug
        %                         %-keyboard
        %                     end
        %                 end
        %             end
        %             % RESTART FROM HERE: see if it's better to have z_aug only as
        %             % the extra-dimensions, to augment it with the point on the
        %             % base space, or a mix of the two.
        %             % for the moment being, we just keep it as a sample in the
        %             % primitive's image space, i.e., the size of the sample is
        %             % given by the dimension of the image space
        %             if mod(ii,push_bias_freq)==0 %&& ~path_found
        %                 z_aug = z_bias(prim.dimensions_imagespace>0); % every once in a while push in a known number
        %                 disp('Pushing in aug goal')
        %                 pushed_in_goal=1;
        %             else
        %                 z_aug = prim.chi.sample;
        %                 %                 z_aug(Ptree.Node{1}.dimensions>0) = z_rand; % BUGFIX DISCONT: only sample in the third dimension, starting from the already added point
        %                 pushed_in_goal=0;
        %             end
        %             while ~CollisionFree(fix_nans([z_aug],Ptree.Node{2}.dimensions_imagespace),Ptree,Obstacles)
        %                 if mod(ii,push_bias_freq)==0 %&& ~path_found
        %                     z_aug = z_bias(prim.dimensions_imagespace>0); % every once in a while push in a known number
        %                     disp('Pushing in aug goal')
        %                     pushed_in_goal=1;
        %                 else
        %                     z_aug = prim.chi.sample;
        %                     %                 z_aug(Ptree.Node{1}.dimensions>0) = z_rand; % BUGFIX DISCONT: only sample in the third dimension, starting from the already added point
        %                     pushed_in_goal=0;
        %                 end
        %             end
        %             % rounding up to the second decimal
        %             z_aug = round(z_aug*100)/100;
        %
        %             Chi_aug = prim.chi;
        %
        %             if verbose
        %                 disp(['z_aug: ' num2str(z_aug(:)')])
        %                 figure(fig_chi0)
        %                 plot(z_rand(1),z_rand(2),'rx','linewidth',2)
        %                 figure(fig_xy);
        %                 plot3(z_aug(1),z_aug(2),z_aug(3),'rx','linewidth',2)
        %                 cprintf('red','Sto per provare con la %s\n',prim.name)
        %             end
        %
        %             idx_parent_primitive = 1;
        %
        %             [T,G,E,z_new_aug,plot_nodes,plot_edges,feasible,added_new,idx_last_added] = localRRTstar(Chi_aug,Ptree,jj,z_aug,T,G,E,Obstacles,verbose,plot_nodes,plot_edges,pushed_in_goal,goal_node,idx_parent_primitive,gam,tol,replicate_over_primitive);
        %             %% set of points with different dimensions along the primitives
        %             if added_new
        %                 %-keyboard
        %                 z_added = T.get(idx_last_added);
        %                 %                 z_new(5)==1 % prim.istask % boolean
        %                 if z_added(5)==1 % TODO: HARDCODED % verify condition on trigger exit
        %                     %-keyboard
        %                     %                     replicate_over_primitive = [replicate_over_primitive, prim.ID];
        %                     replicate_over_primitive = 2;
        %                 end
        %             end
        %
        %             if added_new && reached(T.Node{end},z_goal,tol)
        %                 idz_Goal = T.nnodes; % last one is the goal state, for the moment (in anytime version this will change).
        %                 goal_node = idz_Goal;
        %                 % the -1 is a dirty fix for the fact
        %                 % that this node is inserted two
        %                 % times, once via prim.extend and
        %                 % one from localRRT*(Chi_aug). Does
        %                 % not happen always but still it
        %                 % needs this workaround for those
        %                 % times.
        %                 disp('Goal reached (via Eleva)!');
        %                 path_found = true;
        %                 %         plot(traj_pos,traj_vel,'linewidth',2,'color','yellow')
        %                 if debug
        %                     %-keyboard
        %                 end
        %                 stop = true;
        %                 break
        %             end
        %             if path_found
        %                 [cost,opt_path,~] = graphshortestpath(G,source_node,goal_node);
        %                 opt_path_edges = plot_opt_path(T,opt_path,opt_path_edges);
        %                 % save cost and iteration for plotting (anytime) stuff
        %                 if ~isempty(cost_vector) && cost<cost_vector(end)
        %                     cost_vector = [cost_vector, cost];
        %                     N_cost_vector = [N_cost_vector, ii];
        %                     plot_biograph(source_node,goal_node,G);
        %                     figure(10)
        %                     bar(N_cost_vector,cost_vector); xlabel('Iterations'); ylabel('cost');
        %                     %                     hold on
        %                     save_test_data
        %                     if debug
        %                         %-keyboard
        %                     end
        %                 end
        %             end
        %
        %         end
        %% INTERNAL LOOP ENDED HERE
        
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


%% sending the solution through YARP

if using_yarp
    if(~write_port.isOpen)
        write_port.open('/locomanipulation/solution:o');
    end
    
    cmd = 'solution';
    
    temp_str='';
    for i=1:numel(opt_path)
        for j=1:numel(T.Node{opt_path(i)})
            temp_str = [temp_str ' ' num2str(T.Node{opt_path(i)}(j))];
        end
    end
    temp_str = [cmd ' ' temp_str];
    
    send.fromString(temp_str);
    
    write_port.write(send);
    
    write_port.close;
end


%% simulate the optimal plan that has been found

%% obtain plan as the shortest path in the graph
opt_plan=extract_plan(T,E,opt_path);

%%
save_test_data
%% showtime!
movie = 1;
%% go to test the plan
run_plan
