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

xmin = 0; % it was 0, now it is -10 to allow primitives that start by going backwards and then move onwards
xmax = 10; % meters
ymin = 0;
ymax = 10; % meters
vmin = 0; % linear speed
vmin = -0.5; % linear speed % FIXME: check if this has to be negative too for the coffee application.
vmax = 1; % TODO: check out this value
thetamin = -2*pi;
thetamax = 2*pi;

dimensioni = [1 1 1 1 0]; % only sample in x and v cart.
initial_extend = [0 0 0 0 NaN];
ID = 1; % WARNING: this has to coincide with the index of the primitive in the primitive node! So that by calling primitive.Parent(primitive.ID) returns the primitive's parent. It has to maintain the same order of
% insertion in the tree
% Muovi = Move([xmin vmin; xmin vmax; xmax vmax; xmax vmin],[1 0],cost_table,'Muovi',dimensioni,initial_extend,'blue',ID); % instantiate the primitive Move in Muovi

dimensioni_nuvoletta = [1 1 1 1 0]; % only sample in x cart and y pendulum and v cart.
DD_move_vertices_nuvoletta = [xmin ymin thetamin vmin;
    xmin ymin thetamax vmin;
    xmin ymin thetamin vmax;
    xmin ymin thetamax vmax;
    xmin ymax thetamin vmin;
    xmin ymax thetamax vmin;
    xmin ymax thetamin vmax;
    xmin ymax thetamax vmax;
    xmax ymin thetamin vmin;
    xmax ymin thetamax vmin;
    xmax ymin thetamin vmax;
    xmax ymin thetamax vmax;
    xmax ymax thetamin vmin;
    xmax ymax thetamax vmin;
    xmax ymax thetamin vmax;
    xmax ymax thetamax vmax];

Muovi = DD_move(DD_move_vertices_nuvoletta,[1 0],cost_table,'DD_Muovi',dimensioni,initial_extend,dimensioni_nuvoletta,'blue',ID);
Ptree = Ptree.addnode(idx_primitive_next,Muovi);

idx_primitive_next = idx_primitive_next+1;
if multiple_primitives
    %
    % % xmin = -1;
    % % xmax = +1;
    % % vmin = -5;
    % % vmax = +5;
    xmin_grasping = 6;
    xmax_grasping = 7;
    ymin_grasping = 6;
    ymax_grasping = 7;
    x_target = mean([xmin_grasping,xmax_grasping]);
    y_target = mean([ymin_grasping,ymax_grasping]);
    x_target = 1;
    y_target = 1;
    L_arm = 0.31;
    xmin_grasping = x_target-L_arm/3;
    xmax_grasping = x_target+L_arm/3;
    ymin_grasping = y_target-L_arm/3;
    ymax_grasping = y_target+L_arm/3;
    taumin = 0; % here ymin is the minimum height for the end effector
    taumax = 1;% here ymax is the maximum height for the end effector
    dimensioni = [0 0 0 0 1]; % only sample in x cart and y pendulum and v cart.
    
    initial_extend = [NaN NaN NaN NaN taumin]; % here ymin is the minimum height for the end effector
    ID = ID+1;
    
    dimensioni_nuvoletta = [1 1 0 0 1]; % only sample in x cart and y pendulum and v cart.
    ARM_move_vertices_nuvoletta = [xmin_grasping,ymin_grasping,taumin;
        xmin_grasping ymax_grasping taumin;
        xmin_grasping ymin_grasping taumax;
        xmin_grasping ymax_grasping taumax;
        xmax_grasping ymin_grasping taumin;
        xmax_grasping ymax_grasping taumin;
        xmax_grasping ymin_grasping taumax;
        xmax_grasping ymax_grasping taumax];
    
    Manipulate = ARM_move(ARM_move_vertices_nuvoletta,[1 0],cost_table,'ARM_move',dimensioni,initial_extend,dimensioni_nuvoletta,'green',ID);
    Ptree = Ptree.addnode(idx_primitive_next,Manipulate);
    
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
end
%% Calculate Chi0
% setup initial image space
Chi0 = Ptree.Node{1}.chi; % conventionally in node{1} we have the chi0 space. More generally, here we should check for an intersection of all the image spaces, easy to do with the library we have.
if multiple_primitives
    Chi1 = Ptree.Node{2}.chi;
end
