function T = localRRTstar(Chi,Ptree,x_rand,T)
    prim_cost = zeros(Ptree.nnodes,1);      % cost vector, to choose between different primitives the cheaper one
%% check if other dimensions can be activated from the newest point (x_rand)
    for jj=1:Ptree.nnodes                       % start looking between all available primitives
        jj
        prim = Ptree.get(jj);                   % prim is the current primitive
        %% find the nearest point, in Chi0
        % convert nodes in trees from cells to matrix
        points_mat = cell2mat(T.Node');
        % find nearest point
        idx_nearest = knnsearch(points_mat',x_rand'); % TODO: this can be replaced by a search in the least-cost sense, employing the primitive's cost_table
        x_nearest = T.get(idx_nearest);
        
        x_rand_temp=x_rand;
        x_nearest_temp=x_nearest;
        % TODO: how to represent the space? I suggest sth like sparse matrices
        % with NaNs for non-intersecting (or not yet sampled) dimensions
        %     keyboard
        %waitforbuttonpress
        % TODO: valutare il miglior parametro per muoversi in Chi0
        dimChi0 = Chi.P.Dim;
        dimP = prim.chi.P.Dim;
        
        if dimP > dimChi0
            x_rand_temp(dimChi0+1:dimP) = 0.0;
            x_nearest_temp(dimChi0+1:dimP) = 0.0;
        end
        if prim.chi.P.contains([x_rand_temp, x_nearest_temp])
            prim_cost(jj) = searchCost(prim.cost_table);
            disp(['Found primitive ' prim.getName ' with cost: ' num2str(prim_cost(jj))]);
            %% add the new node to the tree
            T = T.addnode(idx_nearest,x_rand);
            line([x_nearest(1) x_rand(1)],[x_nearest(2) x_rand(2)],'color','red'); % just for visualization
        else
            disp('No primitives found')
            prim_cost(jj) = Inf;
        end
        
    end

    [~,idx_p_opt] = min(prim_cost);
    if prim_cost(idx_p_opt) == Inf
        disp('nessuna primitiva con costo finito disponibile');
    else
        prim_opt = Ptree.get(idx_p_opt);
        disp(['scelgo la primitiva ' prim_opt.getName ' con un costo ' num2str(prim_cost(idx_p_opt))])
    end
    
end

