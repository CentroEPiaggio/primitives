clear all; close all; clc;

V1 = [1 1 0; -1 1 0; -1 -1 0; 1 -1 0;
      1 1 1; -1 1 1; -1 -1 1; 1 -1 1;];
    
P1 = Polyhedron(V1)

V2 = V1*0.5+1;
V2 = V2(1:4,1:2);

P2 = Polyhedron(V2)

%%
x = sdpvar(2,1);
A = eye(2);
vincoli =  [x'*A*x<=1.4];
P3Set = YSet(x,vincoli);
% P3 = P3Set.outerApprox
% P3 = Polyhedron(vincoli)
figure
plot(P3Set)
hold on
plot(P3Set,'color','lightblue')
% plot(P3.chebyCenter,'color','lightgreen')
return
%%
if (P1.Dim ~= P2.Dim)
    minDim = min(P1.Dim,P2.Dim)
    if P1.Dim > minDim
        P1=P1.projection(1:minDim)
    end
    if P2.Dim > minDim
        P2=P2.projection(1:minDim)
    end
    Pintersect = P1 & P2;
    Pintersect.computeVRep;
end

figure
P1.plot('color','red')
% 
hold on
P2.plot('color','blue')

Pintersect.plot('color','lightgreen')
    
F1 = QuadFunction(-[1 0;0 1],[1 1],1)
P1.addFunction(F1,'m(v)')

%%
P1.feval([0;0],'m(v)');

P = Polyhedron('lb',[0;0],'ub',[1; 1])