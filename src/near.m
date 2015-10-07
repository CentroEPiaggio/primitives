% NEAR
% [x_near_bubble,idx_near_bubble] = Near(T,Graph,Edges,cardV)
% INPUT:
% T: the search tree
% Graph: the graph with the costs of connecting the search tree
% Edges: the edges between the nodes in the graph
% cardV: cardinality of the vertices in the graph (i.e. number of nodes)
% OUTPUT:
% x_near_bubble:    the nearest point inside the bubble
% idx_near_bubble:  the index of such point in the tree structure (i.e. to
%                   be able to call Tree.get(idx_near_bubble) to fetch
%                   that node).
% PARAMETERS (to be tuned inside the function):
% gam: a proportionality term on the volume to be considered
function [idX_near,radius] = near(T,Graph,Edges,x_new,cardV)
idX_near=NaN;
radius = 0;
% define the volume of the bubble, according to
% Optimal Kinodynamic Motion Planning using Incremental Sampling-based
% Methods
% keyboard
n = cardV;
if n<2
    idX_near = 1; % return the index of the root of the tree
    radius = 10; % TUNE big value to search in all the searchable space
%     disp('Gimme something more mate!');
        return
end
gam = 1000; % TUNE
volume = gam*log(n)/n; % CHECK: we use natural logarithm here, it is not clear if this is the case.
radius_elevated_n = gamma(1+n/2)/pi^(n/2)*volume;
radius = radius_elevated_n^(1/n);
% BITTER crop the maximum radius at 4.43, see test_radius.m for the reason
% disp('lo printa?')
% radius = min(radius,4.43)

% radius = 100;

% builds a matrix of the nodes (this can be avoided in a future
% implementation with a better data structure)
points_mat = cell2mat(T.Node');
points_mat(isnan(points_mat)) = []; % remove NaN from points
points_mat = reshape(points_mat,2,size(T.Node,1)); % HARDFIX here 2 should be parametrized

% finds the closest points inside the n-ball of volume volume and radius
% radius
% keyboard
idX_near = rangesearch(points_mat',x_new', radius);
idX_near = cell2mat(idX_near);
