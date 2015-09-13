
%% Why do we use a tree? Dan: no particular reason, actually I was thinking
% to a list when first wrote this. Could be on multiple files, or whatever.
% This feels just comfortable to be used (and easy to be translated in a
% C++ data structure. But anything else would work for me here, maybe there
% is something more efficient I did not think about, suggestions are well
% appreciated! ;-)
Ptree = tree; % initialize tree
idx_primitive_next = 0;

cost_table = rand(10,3);

% Forward = PrimitiveFun([-1 -1 0; -1 1 0; 1 -1 0; 1 1 0;-1 -1 1; -1 1 1; 1 -1 1; 1 1 1]*0.3,[1 0],cost_table,'Forward');
dimensioni = [1 1 0]; % [x_carrello v_carrello y_pendolo theta_asta]
xmin = 0;
xmax = 25;
vmin = -10;
vmax = +10;
Muovi = PrimitiveFun([xmin vmin; xmin vmax; xmax vmax; xmax vmin],[1 0],cost_table,'Muovi');
Ptree = Ptree.addnode(idx_primitive_next,Muovi);

idx_primitive_next = idx_primitive_next+1;

xmin = -1;
xmax = +1;
ymin = 1;
ymax = 10;
Abbassa = PrimitiveFun([xmin,ymin; (xmin+xmax)/2,ymax; xmax,ymin],[1 0],cost_table,'Abbassa');
Ptree = Ptree.addnode(idx_primitive_next,Abbassa);

idx_primitive_next = idx_primitive_next+1;
% Left = PrimitiveFun([-1 -1; -1 1; 1 -1; 1 1]*0.4,[10 0],cost_table,'Left');
%TODO
% Alza = PrimitiveFun()
% Ptree = Ptree.addnode(idx_primitive_next,Left)

% idx_primitive_next = idx_primitive_next+1;

% Right = PrimitiveFun([-1 -1; -1 1; 1 -1; 1 1]*0.5,[0.1 0],cost_table,'Right');
% Ptree = Ptree.addnode(idx_primitive_next,Right)
% 
% idx_primitive_next = idx_primitive_next+1;