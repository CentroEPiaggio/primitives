function plot_opt_path(T,opt_path_edges)
fig_xv=2; fig_xy = 3; fig_yv = 4; % stuff to plot

if ~isempty(opt_path_edges)
    for ii=1:size(opt_path_edges,1)
        for jj=1:size(opt_path_edges,2)
            delete(opt_path_edges{ii,jj});
        end
    end
end

end