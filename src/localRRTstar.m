function T = localRRTstar(Chi,Ptree,x_rand,T)
prim_cost = zeros(Ptree.nnodes,1);      % cost vector, to choose between different primitives the cheaper one
prim_feasible = zeros(Ptree.nnodes,1);      % feasibility vector, to check if any feasible primitive has been found
%% check if other dimensions can be activated from the newest point (x_rand)
for jj=1:1%Ptree.nnodes                       % start looking between all available primitives
    jj
    prim = Ptree.get(jj);                   % prim is the current primitive
    %% find the nearest point, in Chi0
    % convert nodes in trees from cells to matrix
%     keyboard
    points_mat = cell2mat(T.Node');
    % now since we want to search in Chi0 we can remove all rows from
    % points_mat that does contain a NaN
%     points_mat(~any(isnan(points_mat)),:)=[]
    points_mat(isnan(points_mat)) = []; % remove NaN from points
    points_mat = reshape(points_mat,2,size(T.Node,1)); % HARDFIX here 2 should be parametrized
    % find nearest pointisp
    idx_nearest = knnsearch(points_mat',x_rand'); % TODO: this can be replaced by a search in the least-cost sense, employing the primitive's cost_table
    x_nearest = T.get(idx_nearest);
    x_nearest(isnan(x_nearest)) = []; % remove NaN
    % nearest point found
    x_rand_temp=x_rand;
    x_nearest_temp=x_nearest;
    % TODO: how to represent the space? I suggest sth like sparse matrices
    % with NaNs for non-intersecting (or not yet sampled) dimensions.
    % UPDATE: this is how it's being implemented right now
    %     keyboard
    %waitforbuttonpress
    % TODO: valutare il miglior parametro per muoversi in Chi0
    dimChi0 = Chi.P.Dim;
    dimP = prim.chi.P.Dim;
    
    if dimP > dimChi0 % add trailing zeros for dimensions outside Chi0
        x_rand_temp(dimChi0+1:dimP) = 0.0;
        x_nearest_temp(dimChi0+1:dimP) = 0.0;
    end
    if prim.chi.P.contains([x_rand_temp, x_nearest_temp])
        %             prim_cost(jj) = searchCost(prim.cost_table); % now we can use
        %             the steering function
        xi = x_nearest_temp(1); vi = x_nearest_temp(2);
        xf = x_rand_temp(1); vf = x_rand_temp(2);
        [feasible,cost]=steering_muovi(xi,xf,vi,vf);
        if feasible
            prim_feasible(jj) = feasible;
            prim_cost(jj) = cost;
            disp(['Found primitive ' prim.getName ' with cost: ' num2str(prim_cost(jj))]);
            %% add the new node to the tree
            x_rand = fix_nans(x_rand,prim.dimensions);
            T = T.addnode(idx_nearest,x_rand);
            line([x_nearest(1) x_rand(1)],[x_nearest(2) x_rand(2)],'color','red'); % just for visualization
        else
            disp('No primitives found')
            prim_feasible(jj) = 0;
            prim_cost(jj) = Inf;
        end
    end
    
end

[~,idx_p_opt] = min(prim_cost);
if prim_cost(idx_p_opt) == Inf
    disp('nessuna primitiva con costo finito disponibile');
elseif feasible
    prim_opt = Ptree.get(idx_p_opt);
    disp(['scelgo la primitiva ' prim_opt.getName ' con un costo ' num2str(prim_cost(idx_p_opt))])
end

end

