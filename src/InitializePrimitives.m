% function [Ptree,Chi0]=InitializePrimitives()

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
vmin = -5;
vmax = +5;
dimensioni = [1 1 0 0]; % only sample in x and v cart.
default_extend = [0 0 NaN NaN];
% Muovi = PrimitiveFun([xmin vmin; xmin vmax; xmax vmax; xmax vmin],[1 0],cost_table,'Muovi',dimensioni,default_extend);
Muovi = Move([xmin vmin; xmin vmax; xmax vmax; xmax vmin],[1 0],cost_table,'Muovi',dimensioni,default_extend); % instantiate the primitive Move in Muovi
Ptree = Ptree.addnode(idx_primitive_next,Muovi);

idx_primitive_next = idx_primitive_next+1;

% xmin = -1;
% xmax = +1;
% vmin = -5;
% vmax = +5;
ymin = 1; % here ymin is the minimum height for the end effector
ymax = 10;% here ymax is the maximum height for the end effector
dimensioni = [1 1 1 0]; % only sample in x cart and y pendulum and v cart.
default_extend = [NaN NaN ymin NaN]; % here ymin is the minimum height for the end effector
% Abbassa = PrimitiveFun([xmin,ymin; (xmin+xmax)/2,ymax; xmax,ymin],[1 0],cost_table,'Abbassa',dimensioni,default_extend);
% Abbassa = PrimitiveFun([xmin,vmin,ymin;
%     xmin vmin ymax;
%     xmin vmax ymin;
%     xmin vmax ymax;
%     xmax vmin ymin;
%     xmax vmin ymax;
%     xmax vmax ymin;
%     xmax vmax ymax],[1 0],cost_table,'Abbassa',dimensioni,default_extend);
Eleva = Elevate([xmin,vmin,ymin;
    xmin vmin ymax;
    xmin vmax ymin;
    xmin vmax ymax;
    xmax vmin ymin;
    xmax vmin ymax;
    xmax vmax ymin;
    xmax vmax ymax],[1 0],cost_table,'Eleva',dimensioni,default_extend);
Ptree = Ptree.addnode(idx_primitive_next,Eleva);

idx_primitive_next = idx_primitive_next+1;

% xmin = 18;
% xmax = 19;
% ymin = 8;
% ymax = 10;
% thetamin = 0;
% thetamax = 1;
% dimensioni = [1 0 1 1]; % sample in x_cart, y_pendulum and pulsante
% Premi = PrimitiveFun([xmin,ymin; xmax,ymin; xmax,ymax; xmin,ymax],[1 0],cost_table,'Premi',dimensioni);
% Ptree = Ptree.addnode(idx_primitive_next,Premi);
% 
% idx_primitive_next = idx_primitive_next+1;

%% Calculate Chi0
% setup initial image space
Chi0 = Ptree.Node{1}.chi; % conventionally in node{1} we have the chi0 space. More generally, here we should check for an intersection of all the image spaces, easy to do with the library we have.
Chi1 = Ptree.Node{2}.chi;
