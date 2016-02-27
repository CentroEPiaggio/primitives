% COMPLETE_TRAJECTORIES
% this function completes the trajectories computed by a primitive to match
% the whole dimension.
% INPUTS:
% - z_start
% - time
% - x
% - Ptree
% - primitive_ID
% - full_state_dimension
% Example:
% if a trajectory has been computed with a locomotion primitive, it will
% affect robot's position, i.e. [x,y].
% Given that there are other dimensions we are sampling on, we might have
% that the whole state is given by [x,y,grasping,valve, etc... ]
% This function returns a vector x_complete where the dimensions computed
% by the primitive with ID primitive_ID are those given by the primitive,
% the others are computed from the starting node with a constant input
% (trimmered) for the same time.
function [x_complete] = complete_trajectories(z_start,time,x,Ptree,primitive_ID)
if size(x,1) == length(Ptree.Node{1}.dimensions)
    x_complete = x;
    return
end

x_complete = zeros(size(x,1),length(time));

for ii=1:Ptree.nnodes
    if ii==primitive_ID % this is the primitive that is active in this moment
        continue
    end
    prim = Ptree.get(ii);
    x_complete(prim.dimensions>0,:) = prim.trim_trajectory(z_start,time,x);
end
end