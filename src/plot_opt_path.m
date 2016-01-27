function new_opt_path_edges = plot_opt_path(T,opt_path,opt_path_edges)
fig_xv=2; fig_xy = 3; fig_yv = 4; % stuff to plot

new_opt_path_edges = {};
for ii=1:length(opt_path)-1
    figure(fig_xv)
    z_start = T.get(opt_path(ii));
    z_end = T.get(opt_path(ii+1));
    edge = line([z_start(1) z_end(1)],[z_start(2) z_end(2)],'color','yellow','linewidth',2,'linestyle',':');
    new_opt_path_edges{1,ii} = edge;
end
% keyboard
if ~isempty(opt_path_edges)
    for ii=1:size(opt_path_edges,1)
        for jj=1:size(opt_path_edges,2)
            delete(opt_path_edges{ii,jj});
        end
    end
end

end