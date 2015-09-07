function T = localRRTstar(P,x_rand,idx_nearest,T)

%% check if other dimensions can be activated from the newest point (x_rand)
    x_nearest = T.get(idx_nearest);
    prim_cost = zeros(P.nnodes,1); % cost vector, to choose between different primitives the cheaper one
    for jj=1:P.nnodes % start looking between all available primitives
        prim = P.get(jj); % prim is the current primitive
        if prim.chi.P.contains([x_rand, x_nearest])
            prim_cost(jj) = searchCost(prim.cost_table);
            disp(['Found primitive ' prim.getName ' with cost: ' num2str(prim_cost(jj))]);
            %% add the new node to the tree
            T = T.addnode(idx_nearest,x_rand);
            % draw arc between newest sample and nearest % TODO: pick best q in the
            % primitive
            x_nearest = T.get(idx_nearest);
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
        prim_opt = P.get(idx_p_opt);
        disp(['scelgo la primitiva ' prim_opt.getName ' con un costo ' num2str(prim_cost(idx_p_opt))])
    end
    
end

