plot_nodes={};
plot_edges={};

if verbose
    fig_chi0 = 2; % figure handle to plot the sampled points and their connections (i.e. graph vertices and edges)
    fig_xy = 3;
    fig_yv = 4;
    % fig_trajectories = 3; % figure handle to plot the sampled points and their trajectories
    figure(fig_chi0)
    Chi0_viz = Polyhedron([xmin, ymin; xmin, ymax; xmax, ymin; xmax ymax])
    Chi0_viz.plot('color','lightgreen','alpha',0.5);hold on;     % plot search region (piano)
    plot(z_init(1),z_init(2),'go','linewidth',4) % plot initial point
    plot(z_goal(1),z_goal(2),'ko','linewidth',4) % plot initial point
    xlabel('x [m]'); ylabel('y [m]'); % TODO: parametrize inside the primitive
    if obstacles_on
        for i=1:numel(Obstacles.Node)
            Obstacles.Node{i}.P.plot('color','black','alpha',0.5);
        end
    end
    figure(fig_xy)
    plot3(z_init(1),z_init(2),0,'go','linewidth',4) % plot initial point % HARDFIX 0 in z to represent tau
    hold on
    grid on
    plot3(z_goal(1),z_goal(2),z_goal(3),'ko','linewidth',4) % plot initial point
    xlabel('x axis [m]'); ylabel('y axis [m]'); zlabel('tau [1]'); % TODO: parametrize inside the primitive
    % plot obstacles
    figure(fig_chi0)
    %     obstacle_speed_limit.P.plot('color','black','alpha',1);
    if multiple_primitives
        figure(fig_xy)
        Chi1.P.plot('color','lightblue','alpha',0.5); hold on;
        if obstacles_on
            obstacle_wall.P.plot('color','black','alpha',0.5);
        end
        for ii=1:length(bias_points)
            plot3(bias_points{ii}(1),bias_points{ii}(2),bias_points{ii}(3),'*','color','yellow','linewidth',10)
        end
    end
    
end


