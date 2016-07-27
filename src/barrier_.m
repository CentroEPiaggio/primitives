clear all; close all; clc;

V1 = [1 1; -1 1; -1 -1; 1 -1];
%     
P1 = Polyhedron(V1);

V2 = [1 1 0; -1 1 0 ; -1 -1 0; 1 -1 0; 
    1 1 1; -1 1 1 ; -1 -1 1; 1 -1 1; ];
P2 = Polyhedron(V2);
% V2 = V1*0.5+1;
% 
% P2 = Polyhedron(V2)
% 
% P1.plot('color','red')
% % 
% hold on
% P2.plot('color','blue')
%%
x = sdpvar(3,1);
A = eye(3);
% vincoli =  [x'*A*x<=0.01];
vincoli = [x(1)^2+x(2)^2<=1; -1<=x(3)<=1];
P3Set = YSet(x,vincoli);
% P3 = P3Set.outerApprox
% P3 = Polyhedron(vincoli)
figure
plot(P3Set)
% hold on
% plot(P3Set,'color','lightblue')
% plot(P3.chebyCenter,'color','lightgreen')
return
%%
Pintersect = P1 & P2;
Pintersect.computeVRep;
Pintersect.plot('color','lightgreen')
    
F1 = QuadFunction(-[1 0;0 1],[1 1],1)
P1.addFunction(F1,'m(v)')

figure
P1.fplot
%%
P1.feval([0;0],'m(v)');

P = Polyhedron('lb',[0;0],'ub',[1; 1])