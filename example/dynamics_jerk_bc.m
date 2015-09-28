function res = dynamics_jerk_bc(ya,yb)
q = [0 10 0 1]; % accelerations shall be 0 at beginning and end of interval
xi = q(1);
xf = q(2);
vi = q(3);
vf = q(4);

res = [ya(1)-xi, ya(2)-vi, ya(3),... % initial conditions on the states
    yb(1)-xf, yb(2)-vf, yb(3) ...  % final conditions on the states
    1+yb(4)*vf + -abs(yb(6))]'; % conditions on the costate is: H(tf)=0 for the free final time problem without final state weight in the figure of merit. Hence it is
% something like: 1+p1(tf)*vf+p2(tf)*af+p3(tf)*u(tf)=0
res