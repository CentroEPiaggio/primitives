load test_a_2016_02_28_16_28_59.mat;

opt_plan=extract_plan(T,E,opt_path)
path_length = opt_plan.nnodes;

t = [];
x = [];
time_offset = 0;
figure
hold on
for ii=2:path_length
    t_ii = time_offset + opt_plan.Node{ii}.time;
    x_ii = opt_plan.Node{ii}.x;
    t = [t, t_ii];
    time_offset = t(end);
    x = [x x_ii];
    plot(t_ii,x_ii);
end
grid on
plot(t(1)*[1 1],z_init(1:2),'ro')
plot(t(end)*[1 1],z_goal(1:2),'ro')

% remind: x is [x,y,theta,v,w]