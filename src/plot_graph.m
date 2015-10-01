function [a1,a2] = plot_graph(T,fig_points,prim,a1,a2,b1,b2)
points = [];
set(a1,'Visible','off')
set(a2,'Visible','off')
set(b1,'Visible','off')
% set(b2,'Visible','off')
for ii=1:length(T.Node)
    points = [points,ii];
    figure(fig_points)
    current_parent=T.Parent(ii);
    if current_parent~=0
        source = T.get(current_parent);
        source=fix_nans(source,prim.dimensions);
        goal = T.get(points(ii));
        goal=fix_nans(goal,prim.dimensions);
        a1=line([source(1) goal(1)],[source(2) goal(2)],'color','blue','linewidth',2); % just for visualization
        a2=plot(goal(1),goal(2),'bo','linewidth',2);
    end
end