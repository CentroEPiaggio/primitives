function state = reached(A,B,tol)
state = 0;
% tol=0.05; % custom % TODO: note that if the goal region is big, like tol=1, it can happen that a goal point more far from its center gives a better cost that one at the center
if isequal(isnan(A),isnan(B)) % check if the two points live in the same subspace
    A = fix_nans(A,~isnan(A));
    B = fix_nans(B,~isnan(B));
    diff = abs(A-B);
    if all( diff < tol*ones(size(diff)) )%1e4*eps(min(abs(A),abs(B)))
        state=1;
    end
    
end