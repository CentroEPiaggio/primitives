function dydt = TPBVPode(t,y)
global A B x0 b a1p;

dydt = y(5)*[A -B*[0 1]/b zeros(2,1); zeros(2,2) -A' zeros(2,1); zeros(1,5)]*y;
