% define obstacles
Obstacles = tree;

speed_limit_bottom = -1;
speed_limit_top = 1;
speed_limit_entry = 10;
speed_limit_exit = 15;
speed_limit_bottom = -5;
speed_limit_top = -3;
speed_limit_entry = 1;
speed_limit_exit = 5;

obstacle_speed_limit = Imagespace(([speed_limit_entry speed_limit_bottom;speed_limit_entry speed_limit_top;speed_limit_exit speed_limit_bottom; speed_limit_exit speed_limit_top]'*1)');
x_wall_min = 10;
x_wall_max = x_wall_min + 5;
% y_wall_min = 2;
y_wall_min = 5;
y_wall_max = y_wall_min + 5;
v_wall_min = -5;
v_wall_max = 5;
obstacle_wall = Imagespace(([ ...
    x_wall_min v_wall_min y_wall_min; x_wall_min v_wall_max y_wall_min; ...
    x_wall_min v_wall_min y_wall_max; x_wall_min v_wall_max y_wall_max; ...
    x_wall_max v_wall_min y_wall_max; x_wall_max v_wall_max y_wall_max; ...
    x_wall_max v_wall_min y_wall_min; x_wall_max v_wall_max y_wall_min]'*1)');
Obstacles = Obstacles.addnode(0,obstacle_speed_limit);
Obstacles = Obstacles.addnode(Obstacles.nnodes,obstacle_wall);
