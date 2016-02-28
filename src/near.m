% NEAR
% [x_near_bubble,idx_near_bubble] = Near(T,Graph,Edges,cardV)
% INPUT:
% T: the search tree
% Graph: the graph with the costs of connecting the search tree
% Edges: the edges between the nodes in the graph
% cardV: cardinality of the vertices in the graph (i.e. number of nodes)
% gam: a proportionality term on the volume to be considered
% OUTPUT:
% x_near_bubble:    the nearest point inside the bubble
% idx_near_bubble:  the index of such point in the tree structure (i.e. to
%                   be able to call Tree.get(idx_near_bubble) to fetch
%                   that node).
function [idX_near,radius] = near(T,Graph,Edges,z_new,z_new_dimensions,cardV,gam)
cprintf('*[0,0,0]*','>>> Enter Near\n');
idX_near=NaN;
radius = 0;
% define the volume of the bubble, according to
% Optimal Kinodynamic Motion Planning using Incremental Sampling-based
% Methods
% keyboard
d = 3; % TUNE: this is the dimension of the whole state space (i.e. the maximum length of a sample vector).
n = cardV;
if n<2
    idX_near = 1; % return the index of the root of the tree
    radius = 10; % TUNE big value to search in all the searchable space
    %     disp('Gimme something more mate!');
    return
end
% gam = 50; % TUNE gam
volume = gam*(log(n)/n).^d; % CHECK: we use natural logarithm here, it is not clear if this is the case.
% radius_elevated_n = gamma(1+n/2)/pi^(n/2)*volume;
% radius = radius_elevated_n^(1/n);
radius = gamma(1+d/2).^(1./d)/sqrt(pi) .* volume.^(1./d);
radius = 10; % HACKONE
global raggio_conta; figure(13); plot(raggio_conta,radius,'bx'); raggio_conta = raggio_conta+1;
% TUNE gam graphically with this
% gam = 100
% n = 1:1e3;
% volume = gam*log(n)./n; % CHECK: we use natural logarithm here, it is not clear if this is the case.
% radius_elevated_n = gamma(1+n/2)./pi.^(n/2).*volume;
% radius = radius_elevated_n.^(1./n);
% figure
% plot(n,volume,n,radius)
%
% BITTER crop the maximum radius at 4.43, see test_radius.m for the reason
% disp('lo printa?')
% radius = min(radius,4.43)

%  radius = 10;
%keyboard
% builds a matrix of the nodes (this can be avoided in a future
% implementation with a better data structure)
points_mat = cell2mat(T.Node');

% idx_same_dimension = find(any(isnan(points_mat(z_new_dimensions>0,:))));
idx_has_nan = any(isnan(points_mat(z_new_dimensions>0,:)));
points_mat(:,idx_has_nan>0) = []
z_new

% finds the closest points inside the n-ball of volume volume and radius
% radius
if ~isempty(points_mat)
    %     keyboard
    idX_near_temp = rangesearch(points_mat(z_new_dimensions>0,:)',z_new', radius);
    idX_near_temp = cell2mat(idX_near_temp);
    if isempty(idX_near_temp)
        % no close point
%         keyboard
        cprintf('*[0,0,0]*','Near: no close point, maybe radius is too small?\n');
        idX_near = []
    else
        idx_compatible = find(~idx_has_nan>0)
        idX_near = idx_compatible(idX_near_temp)
    end
else
    keyboard
    idX_near = [];
end
% rangesearch(points_mat(z_new_dimensions>0)',z_new',radius)
% points_mat(isnan(points_mat)) = []; % remove NaN from points
% points_mat = reshape(points_mat,2,size(T.Node,1)); % HARDFIX here 2 should be parametrized
%
% % finds the closest points inside the n-ball of volume volume and radius
% % radius
% % keyboard
% if ~isempty(points_mat)
%     idX_near = rangesearch(points_mat',z_new', radius);
%     idX_near = cell2mat(idX_near);
% else
%     idX_near = [];
% end
cprintf('*[0,0,0]*','<<< Exit Near\n');