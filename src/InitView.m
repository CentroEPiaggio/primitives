plot_nodes={};
plot_edges={};

if verbose
    fig_chi0 = 2; % figure handle to plot the sampled points and their connections (i.e. graph vertices and edges)
    fig_xy = 3;
    fig_yv = 4;
    % fig_trajectories = 3; % figure handle to plot the sampled points and their trajectories
    figure(fig_chi0)
    Chi0.P.plot('color','lightgreen','alpha',0.5);hold on;     % plot search region (piano)
    plot(z_init(1),z_init(2),'go','linewidth',4) % plot initial point
    plot(z_goal(1),z_goal(2),'ko','linewidth',4) % plot initial point
    xlabel('distance [m]'); ylabel('speed [m/s]'); % TODO: parametrize inside the primitive
    obstacle_speed_limit.P.plot('color','black','alpha',0.5);
    
    figure(fig_xy)
    plot3(z_init(1),z_init(2),1,'go','linewidth',4) % plot initial point % HARDFIX 1 in z_init(3)
    hold on
    plot3(z_goal(1),z_goal(2),z_goal(3),'ko','linewidth',4) % plot initial point
    xlabel('distance [m]'); ylabel('speed [m/s]'); zlabel('pole height'); % TODO: parametrize inside the primitive
    % plot obstacles
    figure(fig_chi0)
%     obstacle_speed_limit.P.plot('color','black','alpha',1);
    if multiple_primitives
        figure(fig_xy)
        Chi1.P.plot('color','lightblue','alpha',0.5); hold on;
        obstacle_wall.P.plot('color','black','alpha',0.5);
        for ii=1:length(bias_points)
            plot3(bias_points{ii}(1),bias_points{ii}(2),bias_points{ii}(3),'*','color','yellow','linewidth',10)
        end
    end
    
end


