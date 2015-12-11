%% calculate control for under-actuated system
B = [1 0 0;
    0 1 0;
    0 0 1];
B = [1 0;
     0 0;
     0 1];
theta_eq_deg = 90;
q_eq = [0,deg2rad(theta_eq_deg),1];
qp_eq = zeros(size(q_eq));
q_eq = q0;
qp_eq = qp0;
theta_eq = q_eq(2);

grav = 9.81;
m = masses;
q = q_eq;

qp = qp_eq;

gpart = [0, 0, 0;
         0, m(3).*grav.*q(3).*sind(rad2deg(theta_eq)), -m(3).*grav.*cosd(theta_eq_deg);
         0, m(3).*grav.*cosd(theta_eq_deg), 0];
% Cpart = [-2.*m(3).*sind(theta_eq_deg).*qp(3).*qp(2)-m(3).*q(3).*cosd(theta_eq_deg).*qp(2)^2;
%     2.*m(3).*q(3).*qp(3).*qp(2)-m(3).*sind(theta_eq_deg).*qp(1).*qp(3)-m(3).*q(3).*cosd(theta_eq_deg).*qp(1).*qp(2);
%     -m(3).*sind(theta_eq_deg).*qp(1).*qp(2)];
Cpart = [0, -m(3).*q(3).*cosd(theta_eq_deg).*qp(2), -2.*m(3).*sind(theta_eq_deg).*qp(2);
         -m(3).*sind(theta_eq_deg).*qp(3), -m(3).*q(3).*cosd(theta_eq_deg).*qp(1), 2.*m(3).*q(3).*qp(2);
         -m(3).*sind(theta_eq_deg).*qp(2), 0, 0];

M = [m(1)+m(3),(-1).*m(3).*q(3).*sind(theta_eq_deg),m(3).*cosd(theta_eq_deg);(-1).*m(3).*q(3) ...
   .*sind(theta_eq_deg),m(3).*q(3).^2,0;m(3).*cosd(theta_eq_deg),0,m(3)];
Minv = inv(M);
% Cpart = zeros(3,3);
Alin = [zeros(3,3), eye(3);
    -Minv*gpart, -Minv*Cpart];
% Blin = [zeros(3,3); Minv*B];
Blin = [zeros(3,2); Minv*B];
Q = 1e3*eye(6);
% R = diag([1 1e9 1]);
R = diag([1 1]);
K = lqr(Alin,Blin,Q,R)
K = place(Alin,Blin,-1:-1:-6)
% add integrator for tracking
eig(Alin-Blin*K)

gravity = [0;
    -m(3).*grav.*q(3).*cosd(theta_eq_deg);
    m(3).*grav.*sind(theta_eq_deg)];

x_eq = [q_eq(:); qp_eq(:)];
tau_eq = -inv(Blin'*Blin)*(Blin'*Alin)*x_eq

Alin*x_eq + Blin*tau_eq

C = [-2.*m(3).*sin(q(2)).*qp(3).*qp(2)-m(3).*q(3).*cos(q(2)).*qp(2)^2;
    2.*m(3).*q(3).*qp(3).*qp(2)-m(3).*sin(q(2)).*qp(1).*qp(3)-m(3).*q(3).*cos(q(2)).*qp(1).*qp(2);
    -m(3).*sin(q(2)).*qp(1).*qp(2)];
S = [1 0;
    0 0;
    0 1];
tau_eq = inv(S'*S)*S'*(C + gravity)
-Minv*(C+gravity-S*tau_eq)