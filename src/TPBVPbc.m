function res=TPBVPbc(ya,yb)
global A B x0 b a1p;

res = [ya(1)-x0(1);
    ya(2)-x0(2);
    yb(1);
    yb(2);
    -0.5*yb(4)^2/b+a1p*yb(5)];
