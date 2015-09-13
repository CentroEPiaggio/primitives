% function [G,W]=make_graph(T)
    parents = T.Parent + 1; % in matlab 1 is the first index of an array
for ii=1:length(parents)
    idx=find(parents==ii)
    cost = T.Node{idx}
    keyboard
%     G(ii,idx) = 
end