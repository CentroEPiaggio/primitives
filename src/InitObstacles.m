% define obstacles
Obstacles = tree;

speed_limit_bottom = (vmin+vmax)/2;
speed_limit_top = vmax;
speed_limit_entry_position = 8;
speed_limit_exit_position = speed_limit_entry_position+5;

obstacle_speed_limit = Imagespace(([speed_limit_entry_position speed_limit_bottom;speed_limit_entry_position speed_limit_top;speed_limit_exit_position speed_limit_bottom; speed_limit_exit_position speed_limit_top]'*1)');
x_wall_min = 10;
x_wall_max = x_wall_min + 5;
% y_wall_min = 2;
y_wall_min = 2;
y_wall_max = y_wall_min + 2;
v_wall_min = vmin;
v_wall_max = vmax;
obstacle_wall = Imagespace(([ ...
    x_wall_min v_wall_min y_wall_min; x_wall_min v_wall_max y_wall_min; ...
    x_wall_min v_wall_min y_wall_max; x_wall_min v_wall_max y_wall_max; ...
    x_wall_max v_wall_min y_wall_max; x_wall_max v_wall_max y_wall_max; ...
    x_wall_max v_wall_min y_wall_min; x_wall_max v_wall_max y_wall_min]'*1)');
Obstacles = Obstacles.addnode(0,obstacle_speed_limit);
Obstacles = Obstacles.addnode(Obstacles.nnodes,obstacle_wall);
