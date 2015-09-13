% map for example test
map_V = [0 10;
    25 10;
    25 0;
    0 0];
obstacle_V = [10 3;
    10 5;
    12 5;
    12 3];
goal_V = [18 8;
    18 10;
    19 10;
    19 8];
map = Polyhedron(map_V);
obstacle=Polyhedron(obstacle_V);
goal = Polyhedron(goal_V);
figure
map.plot('color','cyan','alpha',0.1);
hold on
grid minor
obstacle.plot('color','black');
goal.plot('color','green');