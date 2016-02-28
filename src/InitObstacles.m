% define obstacles
Obstacles = tree;

safety_margin = 0.18;

% x_wall_min = 1.2 - safety_margin;
% x_wall_max = 1.2 + 2.4 + safety_margin;
% % y_wall_min = 2;
% y_wall_min = 0.72 - safety_margin;
% y_wall_max = 0.72 + 0.22 + safety_margin;

x_wall_min = 1 - safety_margin;
x_wall_max = 1 + 0.2 + safety_margin;
% y_wall_min = 2;
y_wall_min = 0 - safety_margin;
y_wall_max = 0.72 + 0.22 + safety_margin;

dimensions_wall = [1 1 0 0]; % walls affect x and y dimensions
obstacle_wall = Obstacle(([ ...
    x_wall_min y_wall_min; ...
    x_wall_min y_wall_max; ...
    x_wall_max y_wall_max; ...
    x_wall_max y_wall_min]'*1)', dimensions_wall);
Obstacles = Obstacles.addnode(0,obstacle_wall);
%
% speed_limit_bottom = (vmin+vmax)/2;
% speed_limit_top = vmax;
% speed_limit_entry_position = 8;
% speed_limit_exit_position = speed_limit_entry_position+5;
%
% obstacle_speed_limit = Imagespace(([speed_limit_entry_position speed_limit_bottom;speed_limit_entry_position speed_limit_top;speed_limit_exit_position speed_limit_bottom; speed_limit_exit_position speed_limit_top]'*1)');
% x_wall_min = 10;
% x_wall_max = x_wall_min + 5;
% % y_wall_min = 2;
% y_wall_min = 2;%4
% y_wall_max = y_wall_min + 2;
% v_wall_min = vmin;
% v_wall_max = vmax;
% obstacle_wall = Imagespace(([ ...
%     x_wall_min v_wall_min y_wall_min; x_wall_min v_wall_max y_wall_min; ...
%     x_wall_min v_wall_min y_wall_max; x_wall_min v_wall_max y_wall_max; ...
%     x_wall_max v_wall_min y_wall_max; x_wall_max v_wall_max y_wall_max; ...
%     x_wall_max v_wall_min y_wall_min; x_wall_max v_wall_max y_wall_min]'*1)');
% Obstacles = Obstacles.addnode(0,obstacle_speed_limit);
% Obstacles = Obstacles.addnode(Obstacles.nnodes,obstacle_wall);
