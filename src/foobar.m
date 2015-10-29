source_node = 1;
goal_node = 10% idz_Goal
[opt_path,cost]=plot_biograph(source_node,goal_node,G)
opt_plan=extract_plan(T,E,opt_path)
%%
run_plan

%%
figure,
tt = q_reference(1,:)
plot(tt,q_reference(2,:),tt,q_reference(3,:))
title('vel')
posx = cumtrapz(tt,q_reference(2,:));
posy = cumtrapz(tt,q_reference(3,:));

figure
plot(tt,posx,tt,posy)
title('pos')

%%
figure
plot(opt_plan.Node{2}.time,opt_plan.Node{2}.x, ...
    10+opt_plan.Node{3}.time,opt_plan.Node{3}.x, ...
     20+opt_plan.Node{4}.time,opt_plan.Node{4}.x, ...
    30+opt_plan.Node{5}.time,opt_plan.Node{5}.x)
legend('pos','vel','height')
grid on

q_reference = [0;0;0];
% loop
figure
for kk=2:opt_plan.nnodes
    time = opt_plan.Node{kk}.time;
    traj_x_speed_cart = opt_plan.Node{kk}.x(2,:);
    traj_y_speed_cart = gradient(opt_plan.Node{kk}.x(3,:))/mean(diff(opt_plan.Node{kk}.time));
%     if strcmp(opt_plan.Node{kk-1}.primitive,'Eleva') && strcmp(opt_plan.Node{kk}.primitive,'Muovi')
%         disp('Eleva after Muovi. What''s going on?')
%         keyboard
%     end
    q_reference_add = [q_reference(1,end)+time(:)'; % conventionally SIMULINK requires that the first row of the vector q_reference (used in a from_file block) is the time
        traj_x_speed_cart(:)';
        traj_y_speed_cart(:)'];
    q_reference = [q_reference, q_reference_add];
    plot(q_reference(1,:),q_reference(2:end,:))
    hold on
%     keyboard
end
legend('vel cart','vel height')
grid on