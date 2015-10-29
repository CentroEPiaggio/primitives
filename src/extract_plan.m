function opt_plan = extract_plan(T,E,path)
opt_plan = tree;
prim_name = 'Standby';
prim_params = 0;
pi_I = struct('primitive',prim_name, ...
    'primitive_q',prim_params);
opt_plan = opt_plan.addnode(0,pi_I);
for ii=2:length(path)
    idx_child = path(ii);
    idx_parent = T.Parent(idx_child); % because tree class uses 0 as starting index
    opt_plan=opt_plan.addnode(ii-1,E{idx_parent,idx_child});
end

return
%%
z_init = T.Node{1};
x_values=z_init(1);
y_values=z_init(2);
for k=2:3%length(opt_plan.Node) % HARDFIX: formally correct but it has to be generalized
    if isequal(opt_plan.Node{k}.primitive,'Muovi')
%         x_values = horzcat(x_values,opt_plan.Node{k}.primitive_q(2));
        x_values = horzcat(x_values,opt_plan.Node{k}.x(2,:));
        y_values = horzcat(y_values,y_values(end));
    elseif isequal(opt_plan.Node{k}.primitive,'Eleva')
        x_values = horzcat(x_values,x_values(end));
%         y_values = horzcat(y_values,opt_plan.Node{k}.primitive_q(2));
        y_values = horzcat(y_values,opt_plan.Node{k}.x(2,:));
    end
end

keyboard