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
function [idx_nearest,z_nearest] = Nearest(z_rand,T,prim,parent_prim)
%% find the nearest point, in the current Chi
% convert nodes in trees from cells to matrix keyboard
% keyboard
debug = 1;
if prim.ID == 2 && debug
%     keyboard
end

points_mat = cell2mat(T.Node');

z_test = fix_nans(z_rand,prim.dimensions_imagespace);
if z_test(5) == 1 %z_test(5) == 0 || z_test(5) == 1
%     keyboard
end
    
    
if prim.ID == 1 % TODO: HARDCODED
    temp = points_mat(~isnan(prim.dimensions_imagespace==0),:);
    test1 = temp(prim.dimensions_imagespace==0,:);
    
    test2 = kron(z_test(prim.dimensions_imagespace==0),ones(1,size(points_mat,2)))
    keep_columns = [];
    for ii=1:length(test1)
        if isequaln(test1(ii),test2(ii))
            keep_columns = [keep_columns, ii];
        end
    end
    points_mat_purged = points_mat(:,keep_columns)
else
%     keyboard
    keep_columns = 1:size(points_mat,2);
    points_mat_purged = points_mat;
end
mask = ~isnan(z_test)
points_mat_masked = points_mat_purged(mask>0,:);
non_nans_col = find(all(~isnan(points_mat_purged(mask>0,:))));
try
    points_mat_masked = points_mat(mask>0,keep_columns(non_nans_col));
    idx_nearest_temp = knnsearch(points_mat_masked',z_rand(mask>0)');
    idx_nearest = keep_columns(non_nans_col(idx_nearest_temp));
    z_nearest = T.get(idx_nearest);
catch ME
    disp(ME.message)
    keyboard
end
return

% by definition, the closer nearest point has to be inside the same image space.
% I.e. no distance defined outside the primitive space.
%  non_nans=[];
%  for i=1:length(prim.dimensions)
%      if prim.dimensions(i)>0
%          non_nans = [non_nans i];
%      end
%  end
if ~isempty(parent_prim)
    non_nans_dimensions = (prim.dimensions_imagespace>0)% & (parent_prim.dimensions_imagespace>0);
else
    non_nans_dimensions = (prim.dimensions_imagespace>0);
end
non_nans=[];
for i=1:length(prim.dimensions)
    if non_nans_dimensions(i)>0
        non_nans = [non_nans i];
    end
end
points_mat_non_nans = points_mat(non_nans,:);
% % TODO: this is no longer needed, right?
% if length(non_nans)~=length(z_rand)
%     keyboard
% end
% z_rand = z_rand(non_nans);
try
    z_rand_temp = fix_nans(z_rand,prim.dimensions_imagespace);
    z_rand = z_rand_temp(non_nans);
catch ME
    disp(ME.message);
    keyboard
end
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
