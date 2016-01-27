%RECONNECT
% [T,G,E] = ReConnect(idx_new,idx_near,T,G,E,prim,q,cost)
function [T,G,E,pn,pe] = ReConnect(idx_new,idx_near,T,G,E,prim,q,cost, x, time, pn,pe,fig_points)
cprintf('*[0,0,0]*','>>> Enter ReConnect\n');
keyboard

z_near = T.get(idx_near);
z_near = fix_nans(z_near,prim.dimensions);
% re-insert the z_near node attaching it to the z_new node
% instead of InsertNode try to play with the parents of the node
% assigning a new parent to an existing node
idx_old = T.getparent(idx_near);
T.Parent(idx_near) = idx_new;
% remove old edge
E{idx_old,idx_near} = [];
% keyboard % use plot_biograph(1,T.nnodes,G) to visualize rewiring. Call it
% here and after G(idx_new,idx_near) = cost;
G(idx_old,idx_near) = 0;
% set(pe(idx_near),'visible','off');
% delete(pe(idx_near));
delete(pe{1,idx_near});
% delete(pe{2,idx_near}); % TODO on vertical graph

G(idx_new,idx_near) = cost;
actions = struct('source_node', idx_new,...
        'dest_node', idx_near,...
        'primitive',prim.name,...
        'primitive_q',q,...
        'x',x,...
        'time',time);
E{idx_new,idx_near} = actions;

z_new = T.get(idx_new);
figure(fig_points)
%node = plot(z_new(1),z_new(2),'bo','linewidth',2);
%plot_nodes = horzcat(plot_nodes,node);
edge = line([z_new(1) z_near(1)],[z_new(2) z_near(2)],'color','magenta','linewidth',2); 
pe{1,idx_near}=edge; % THIS SEEMS TO BE RIGHT

cprintf('*[0,0,0]*','<<< Exit ReConnect\n');