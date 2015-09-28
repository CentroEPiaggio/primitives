function dy = dynamics_jerk(t,y)
% keyboard
A = [0 1 0;
    0 0 1;
    0 0 0];
B = [0; 0; 1];
u = -sign(y(6)); % optimal control
Atot = [A zeros(size(A)) ;
    zeros(size(A')), -A'];
Atot(end+1,end+1)=0;
Btot = [B;0;0;0;0];
dy = Atot*y + Btot*u;
T = y(end);
dy = T*dy; % time rescaling
dy