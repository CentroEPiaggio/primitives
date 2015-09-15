dimensioni = [0 0 0 0]; % [x_carrello v_carrello y_pendolo pulsante]
% the vector dimensioni could be built automatically each time a primitive
% is added to the tree. This is surely an improvement to make. For the time
% being (FTTB) there are more urgent things to do. The solution implemented here
% should be enough FTTB.
% usage: just put a 1 if a primitive has image space in that dimension, or
% 0 otherwise.
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

xmin = -2; % it was 0, now it is -10 to allow primitives that start by going backwards and then move onwards
xmax = 25;
vmin = -10;
vmax = +10;
dimensioni = [1 1 0 0]; % only sample in x and v cart.
Muovi = PrimitiveFun([xmin vmin; xmin vmax; xmax vmax; xmax vmin],[1 0],cost_table,'Muovi',dimensioni);
Ptree = Ptree.addnode(idx_primitive_next,Muovi);

idx_primitive_next = idx_primitive_next+1;

xmin = -1;
xmax = +1;
ymin = 1;
ymax = 10;
dimensioni = [1 0 1 0]; % only sample in x cart and y pendulum.
Abbassa = PrimitiveFun([xmin,ymin; (xmin+xmax)/2,ymax; xmax,ymin],[1 0],cost_table,'Abbassa',dimensioni);
Ptree = Ptree.addnode(idx_primitive_next,Abbassa);

idx_primitive_next = idx_primitive_next+1;

xmin = 18;
xmax = 19;
ymin = 8;
ymax = 10;
thetamin = 0;
thetamax = 1;
dimensioni = [1 0 1 1]; % sample in x_cart, y_pendulum and pulsante
Premi = PrimitiveFun([xmin,ymin; xmax,ymin; xmax,ymax; xmin,ymax],[1 0],cost_table,'Premi',dimensioni);
Ptree = Ptree.addnode(idx_primitive_next,Premi);

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