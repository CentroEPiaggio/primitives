% >>> Enter ReConnect
% Entered inside InsertNode
% Using primitive: Eleva
% Added node: {41} as [5.41         0.5        1.68]. Parent: {40}.
% From node {40}: [4.45         0.5        1.57] or {40}
% To   node {41}: [5.41         0.5        1.68] or {41}
% ReConnect-ed adding an edge from node 40 to a new intermediate node 41
% ReConnect-ed from new node 41 to old node 9
% ReConnect-ed from new node 40 to old node 9
% <<< Exit ReConnect
% 
% Discontinuity in the edge from node 41 to node 9


%%

nodestart = 40;
intermediatenode = 41;
nodeend = 32;
figure,plot(E{nodestart,intermediatenode}.time,E{nodestart,intermediatenode}.x,E{intermediatenode,nodeend}.time+E{nodestart,intermediatenode}.time(end),E{intermediatenode,nodeend}.x)
hold on
plot(E{nodestart,intermediatenode}.time(1)*ones(3,1), T.get(nodestart),'ro')
plot(E{nodestart,intermediatenode}.time(end)*ones(3,1), T.get(intermediatenode),'ro')
plot((E{nodestart,intermediatenode}.time(end)+ E{intermediatenode,nodeend}.time(end))*ones(3,1), T.get(nodeend),'ro')
