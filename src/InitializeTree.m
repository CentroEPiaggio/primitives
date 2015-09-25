function [T,G,E] = InitializeTree()
% initialize an empty tree
T = tree;
% empty search graph
G = sparse(1,1,0); % cost graph
E = cell(1,1);     % primitive graph

