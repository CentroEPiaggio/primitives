plot_nodes=0;
plot_edges=0;

if verbose
    fig_chi0 = 2; % figure handle to plot the sampled points and their connections (i.e. graph vertices and edges)
    fig_xy = 3;
    fig_yv = 4;
    % fig_trajectories = 3; % figure handle to plot the sampled points and their trajectories
    figure(fig_chi0)
    Chi0.P.plot('color','lightgreen','alpha',0.5);hold on;     % plot search region (piano)
    % Chi1.P.plot('color','lightblue','alpha',0.5)
    %     axis equal;
    plot(z_init(1),z_init(2),'go','linewidth',4) % plot initial point
    plot(z_goal(1),z_goal(2),'ko','linewidth',4) % plot initial point
    figure(fig_xy)
    plot3(z_init(1),z_init(2),1,'go','linewidth',4) % plot initial point % HARDFIX 1 in z_init(3)
    hold on
    plot3(z_goal(1),z_goal(2),z_goal(3),'ko','linewidth',4) % plot initial point
    % figure(fig_trajectories)
    % Chi0.P.plot('color','lightgreen','alpha',0.5);hold on;     % plot search region (piano)
    % axis equal;
    % plot(z_init(1),z_init(2),'go','linewidth',4) % plot initial point
    % plot(z_goal(1),z_goal(2),'ro','linewidth',4) % plot initial point
    
    
    % plot obstacles
    figure(fig_chi0)
    obstacle_speed_limit.P.plot('color','black','alpha',1);
    if multiple_primitives
        figure(fig_xy)
        Chi1.P.plot('color','lightblue','alpha',0.5); hold on;
        obstacle_wall.P.plot('color','black','alpha',1);
    end
    
end


