function [path,dist]=plot_biograph(source_node,goal_node,G)
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
%% NON VA ROBA DI COLORI
edges = getedgesbynodeid(h,get(h.Nodes(path),'ID'));
set(edges,'LineColor',[1 0 0])
set(edges,'LineWidth',1.5)
