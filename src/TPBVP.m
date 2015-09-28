function m = TPBVP(p1,p2)
global A B x0 b a1p;

A = [0 1; 0 0];
B = [0 1]';
x0 = [10 0]';
b = p1;
a1p = p2;

solinit = bvpinit(linspace(0,1),@TPBVPinit);
sol = bvp4c(@TPBVPode,@TPBVPbc,solinit);

time = sol.y(5)*sol.x;
state = sol.y([1 2],:);
adjoint = sol.y([3 4],:);
control = -(1/b)*sol.y(4,:);
m(1,:)=time;
m([2 3],:) = state;
m([4 5],:) = adjoint;
m(6,:) = control;

keyboard