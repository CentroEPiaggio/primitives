function res = dynamics_jerk_bc(ya,yb)
% q = [0 10 0 0]; % accelerations shall be 0 at beginning and end of interval
% xi = q(1);
% xf = q(2);
% vi = q(3);
% vf = q(4);
% 
% xa = ya(1:4); xb = yb(1:4);
% pa = ya(5:8); pb = yb(5:8);
% 
% A = [0 1 0;
%     0 0 1;
%     0 0 0];
% B = [0;0;1];
% T = yb(4); % tempo finale
% n = size(A,1);
% TA = T*[A,zeros(n,1);zeros(1,n+1)];
% TB = [B;0];
% % keyboard
% res = [xa(1)-xi, xa(2)-vi, xa(3),... % initial conditions on the states
%     xb(1)-xf, xb(2)-vf, xb(3), ...  % final conditions on the states
%     pb'*TB, ...
%     1+pb'*TA*xb];
% %     1+yb(4)*vf + -abs(yb(6))]'; % conditions on the costate is: H(tf)=0 for the free final time problem without final state weight in the figure of merit. Hence it is
% % something like: 1+p1(tf)*vf+p2(tf)*af+p3(tf)*u(tf)=0
res = [ya(1)-2;ya(2)-2;yb(1);yb(2);yb(3)^2+yb(4)^2-1];
res = res(:)