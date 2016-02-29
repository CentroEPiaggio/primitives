% FIX_NANS
% this function is used to place NaNs in a sampled vector in those
% positions (i.e. at those indices) where we have not sampled.
% example: we want to sample in R^2 but the overall space is R^3:
% the sampled vector x_sampled = [4 7] is in R^2.
% by calling x=fix_nans(x_sampled,[1 1 0])
% we return x=[4 7 NaN];
% or by calling x=fix_nans(x_sampled,[1 0 1])
% we return x=[4 NaN 7];
%
% In the opposite case we just remove NaNs from the vector,
% e.g. x_sampled = [4 NaN 7]
% we return x=[4 7];
%
% In the case we have x_sampled = [4 7 2] and the primitive acts only on a
% subset of the already non-NaN dimensions, we fix only the non-initialized dimensions:
% e.g. x_sampled = [4 7 2]
% x=fix_nans(x_sampled,[1 1 0 0])
% we return x=[4 7 2 NaN];
function x = fix_nans(x_sampled,dimensions)
% keyboard
x = zeros(length(dimensions),1);
if nargin==1 % obtain non-NaN dimensions
    x = x_sampled(~isnan(x_sampled)>0);
    return
end

if length(x_sampled)<length(dimensions) % we have been asked to add nans in proper place
    idx=~dimensions;
    x = nan(size(dimensions));
    if sum(~idx) == length(x_sampled)
        x(~idx) = x_sampled;
    else
        x(idx(1:length(x_sampled))) = x_sampled(idx(1:length(x_sampled)));
        x(~idx) = x_sampled(~idx);
    end
    x = x(:); % column vector
else % we have been asked to remove nans from a vector
    if nargin==2 % non-NaN dimensions passed as argument
        x(dimensions>0) = x_sampled(dimensions>0);
        x(~dimensions) = x_sampled(~dimensions); % this ensures that non-nan dimensions outside dimension>0 are kept to their current value
    end
end
end

%% FIX THIS STUFF
% fix_nans([4 7]',[1 1 0 1])
% In an assignment  A(I) = B, the number of elements in B and I must be the same.
%
% Error in fix_nans (line 23)
%     x(~idx) = x_sampled;
%
% fix_nans([4 7],[1 1 0 1])
% In an assignment  A(I) = B, the number of elements in B and I must be the same.
%
% Error in fix_nans (line 23)
%     x(~idx) = x_sampled;
%
% fix_nans([4 NaN 7])
%
% ans =
%
%      4     7
%
% fix_nans([4 7],[0 0 1])
% In an assignment  A(I) = B, the number of elements in B and I must be the same.
%
% Error in fix_nans (line 23)
%     x(~idx) = x_sampled;
%
% fix_nans([4 7],[1 1 1])
% In an assignment  A(I) = B, the number of elements in B and I must be the same.
%
% Error in fix_nans (line 23)
%     x(~idx) = x_sampled;
%
% fix_nans([4 7],[1 1 0])
%
% ans =
%
%      4
%      7
%    NaN
%
% fix_nans([4 7],[1 1 0 0])
%
% ans =
%
%      4
%      7
%    NaN
%    NaN
%
% fix_nans([4 7],[1 1 0 0 1])
% In an assignment  A(I) = B, the number of elements in B and I must be the same.
%
% Error in fix_nans (line 23)
%     x(~idx) = x_sampled;
%
% fix_nans([4 7],[1 1 0 0 0])
%
% ans =
%
%      4
%      7
%    NaN
%    NaN
%    NaN
%
% fix_nans([4 NaN 7],[1 0 1 0 0 0])
% In an assignment  A(I) = B, the number of elements in B and I must be the same.
%
% Error in fix_nans (line 23)
%     x(~idx) = x_sampled;
%
% fix_nans([4 NaN 7],[1 1 1 0 0 0])
%
% ans =
%
%      4
%    NaN
%      7
%    NaN
%    NaN
%    NaN
%
