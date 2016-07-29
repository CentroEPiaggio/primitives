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
% if ~any(isnan(x(:,1)))
%     x_complete = x;
%     return
% end
if size(x,1) == length(Ptree.Node{1}.dimensions) && isequaln(z_start(Ptree.Node{1}.dimensions==0), x(Ptree.Node{1}.dimensions==0,1))
    x_complete = x;
    return
end

if Ptree.nnodes > 1
    x_complete = zeros(length(Ptree.Node{1}.dimensions),length(time));
else
    x_complete = zeros(sum(Ptree.Node{1}.dimensions),length(time));
end

for ii=1:Ptree.nnodes
    prim = Ptree.get(ii);
%     keyboard
    if ii==primitive_ID % this is the primitive that is active in this moment
        x_complete(prim.dimensions>0,:) = x;
        continue
    end
    x_complete(prim.dimensions>0,:) = prim.trim_trajectory(z_start,time,x_complete);
end
end