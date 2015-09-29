function a=plot_graph(T,fig_points,prim,a)
points = [];
delete a;
for ii=1:length(T.Node)
    points = [points,ii];
    figure(fig_points)
    current_parent=T.Parent(ii);
    if current_parent~=0
        source = T.get(current_parent);
        source=fix_nans(source,prim.dimensions);
        goal = T.get(points(ii));
        goal=fix_nans(goal,prim.dimensions);
        a=line([source(1) goal(1)],[source(2) goal(2)],'color','blue','linewidth',2); % just for visualization
        plot(goal(1),goal(2),'mx','linewidth',2)
    end
end