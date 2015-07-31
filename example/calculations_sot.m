clear all; close all; clc;

syms d th;
J = [1, -d*sin(th), cos(th);
    0, d*cos(th), sin(th);
    0, 1, 0]

rank(J)

Jpinv = J.'*inv(J*J.');
latex(simplify(Jpinv))

c = J*J.'

%%
pretty(simplify(Jpinv))
pretty(simplify(inv(J)))

%% First task
J1 = [1 0 0];
J1_pinv = J1.'*inv(J1*J1.')
%% Second task
J2 = [1, -d*sin(th), cos(th);
    0, d*cos(th), sin(th)];
JA_1 = J1;
PA_1 = eye(3)-JA_1.'*inv(JA_1*JA_1.')*JA_1
Jbar_2 = J2*PA_1
Jbar_2_pinv = simplify(Jbar_2.'*inv(Jbar_2*Jbar_2.'))