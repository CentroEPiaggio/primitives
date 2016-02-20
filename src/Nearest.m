%NEAREST: find the point in the tree which is nearest to a given
% point.
% [idx_nearest,z_nearest,dim_nearest] = nearest(x_rand,T)
% arguments:
% - x_rand: the given point
% - T:      the tree
% returns:
% - idx_nearest:    index of the nearest in the tree
% - z_nearest:      the nearest point
% - dim_nearest:    the dimension vector of the nearest point
% function [idx_nearest,z_nearest,dim_nearest] =
% nearest(z_rand,T,dimensions_z_rand,dimensions_chi0) % no need of
% dim_nearest, we can just keep the NaNs.
function [idx_nearest,z_nearest] = Nearest(z_rand,T,chi0)
%% find the nearest point, in the current Chi
% convert nodes in trees from cells to matrix keyboard
% keyboard
points_mat = cell2mat(T.Node');

% by definition, the closer nearest point has to be inside the same image space. 
% I.e. no distance defined outside the primitive space.
non_nans=[];
for i=1:length(chi0.dimensions)
    if chi0.dimensions(i)>0
        non_nans = [non_nans i];
    end
end
points_mat_non_nans = points_mat(non_nans,:);
if length(non_nans)~=length(z_rand)
    keyboard
end
z_rand = z_rand(non_nans);
% now points_mat_z_rand_dimension cointains only the points that live
% in the same space of z_rand (i.e. have the same non-NaN dimensions)
col_temp=any(isnan(points_mat_non_nans));
non_nans_col=[];
for i=1:length(col_temp)
    if ~col_temp(i)
        non_nans_col = [non_nans_col i];
    end
end
points_mat_non_nans_col = points_mat_non_nans(:,non_nans_col);
idx_nearest_temp = knnsearch(points_mat_non_nans_col',z_rand');
idx_nearest = non_nans_col(idx_nearest_temp);
z_nearest = T.get(idx_nearest);
