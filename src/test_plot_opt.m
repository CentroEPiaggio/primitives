% make the sparse matrix square % is this really needed here?
GGG = G;
sizeG = size(GGG);
[~,shorterDim]=min(sizeG);
GGG(sizeG(shorterDim)+1:max(sizeG),:)=0;

[cost,opt_path,~] = graphshortestpath(GGG,1,T.nnodes);
opt_path_edges = plot_opt_path(T,opt_path,opt_path_edges);
keyboard
clean_path(T,opt_path_edges)