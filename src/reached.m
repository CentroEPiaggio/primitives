function state = reached(A,B)
state = 0;
tol=1;%0.05; % custom
if isequal(isnan(A),isnan(B)) % check if the two points live in the same subspace
    A = fix_nans(A,~isnan(A));
    B = fix_nans(B,~isnan(B));
    diff = abs(A-B);
    if all( diff < tol*ones(size(diff)) )%1e4*eps(min(abs(A),abs(B)))
        state=1;
    end
    
end