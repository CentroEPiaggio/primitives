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
function x = fix_nans(x_sampled,dimensions)
if length(x_sampled)<length(dimensions) % we have been asked to add nans in proper place
    idx=~dimensions;
    x = nan(size(dimensions));
    x(~idx) = x_sampled;
    x = x(:); % column vector
else % we have been asked to remove nans from a vector
    x = x_sampled(dimensions>0);
end
end