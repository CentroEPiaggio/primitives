function idx_min = ChooseParent(idX_near, idx_nearest, T, G, x_new, cost_x_new)

% make the sparse matrix square
sizeG = size(G);
[~,shorterDim]=min(sizeG);
G(sizeG(shorterDim)+1:max(sizeG),:)=0;

idx_min = idx_nearest;
%CHOOSEPARENT determines the best parent in the cost sense
idx_I = 1;
% x_new = T.get(idx_new);
x_nearest = T.get(idx_nearest);
c_new = Inf;

% costo = costo accumulato + costo nuovo campione
c_x_nearest = graphshortestpath(G,idx_I,idx_nearest);
c_min = c_x_nearest + cost_x_new;  

for i=1:length(idX_near) % for every point btw the near by vertices
    if ~isempty(idX_near{i})
        X_near(i,:)=T.get(idX_near{i});
        [feasible,cost_actual,q,traj_pos,traj_vel] = steering_muovi(X_near(i,1),x_new(1),X_near(i,2),x_new(2));
        if feasible
%             if any(Obstacles.Node{1}.P.contains([traj_pos(:)'; traj_vel(:)'])) % ObstacleFree
                % costo fino al near
                c_x_near = graphshortestpath(G,idx_I,idX_near{i});
%                 c_x_new = graphshortestpath(G,idx_I,idx_new);
                % costo fino al near + pezzettino near-new
                c_actual = c_x_near + cost_actual;
                if (c_actual < c_min)% && (c_actual < c_x_new)
                    idx_min = idX_near{i};
                    c_min = c_actual;
                    disp('AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA')
                end
%             end
        end
    end
end
end


