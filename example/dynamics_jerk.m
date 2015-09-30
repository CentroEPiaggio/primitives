function dy = dynamics_jerk(t,y)
% % keyboard
% A = [0 1 0;
%     0 0 1;
%     0 0 0];
% B = [0; 0; 1];
% % u = -sign(y(6)); % optimal control
% % Atot = [A zeros(size(A)) ;
% %     zeros(size(A')), -A'];
% % Atot(end+1,end+1)=0;
% % Btot = [B;0;0;0;0];
% % dy = Atot*y + Btot*u;
% % T = y(end);
% % dy = T*dy; % time rescaling
% % dy
% T = y(4)
% n = size(A,1);
% TA = T*[A,zeros(n,1);zeros(1,n+1)];
% TB = T*[B;0];
% AA = [TA,zeros(size(TA));
%     zeros(size(TA)), -TA'];
% BB = [TB; zeros(size(TB))];
% 
% % u = -sign(y(end)); % y(end) = p_n
% % u = -tanh(y(end));
% u = -tanh(y(end)/1e-12);
% 
% dy = AA*y+BB*u;

T = y(5);
dy1 = T*y(2);
dy2 = T*tanh(y(4)/1e-7);

muu=0.5;
dy1 = T*(y(2)+muu*y(3)/sqrt(muu*y(3)^2+y(4)^2));
dy2 = T*(y(4)/sqrt(muu*y(3)^2+y(4)^2));
dy3 = 0;
dy4 = -T*y(3);
dy5 = 0;
dy = [dy1;dy2;dy3;dy4;dy5];