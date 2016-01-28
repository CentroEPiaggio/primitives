function [z_truncated, x_truncated] = truncate_to_similar(z_original,x)
tol = 1e9;
z_truncated = z_original;
x_truncated = x;
% keyboard % WARNING WE SHOULD NOT ENTER HERE ANYMORE
cprintf('*[0,0,0]*','||| WARNING WE SHOULD NOT ENTER HERE ANYMORE |||\n');
for ii=1:length(z_original)
    z_original_tol = round(z_original(ii)*tol)/tol;
    x_final_tol = round(x(ii,end)*tol)/tol;
    if z_original_tol==x_final_tol
        z_truncated(ii) = z_original_tol;
        x_truncated(ii,end) = z_original_tol;
    end
end
end