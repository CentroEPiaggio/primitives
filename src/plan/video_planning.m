% load test_a_2016_03_02_07_31_06.mat;
load test_2016_07_28_03_09_00.mat;
%%
close all

InitView

close(fig_chi0)

frames=1;
formatOut = 'yyyy_mm_dd_hh_MM_SS';
str_date=datestr(now,formatOut);
vidObj=VideoWriter(['plan_' str_date '.avi']);
vidObj.Quality = 100;
desired_time = 7; %sec
vidObj.FrameRate = numel(T.Node)/desired_time;
open(vidObj);

view([-50 30])
hold on

movie_anima(frames) = getframe(figure(fig_xy));
frames = frames +1;

for i=1:numel(T.Node)
    node = T.Node{i};
    if isnan(node(5))
        node(5)=0;
    end
    plot3(node(1),node(2),node(5),'rx','linewidth',2);

    parent_id = T.Parent(i);
    if parent_id==0
        continue
    end
    parent = T.Node{parent_id};
    if isnan(parent(5))
        parent(5)=0;
    end
    
    plot3([node(1) parent(1)],[node(2) parent(2)],[node(5) parent(5)],'b','linewidth',2);

    movie_anima(frames) = getframe(figure(fig_xy));
    frames = frames +1;
end

opt_plan=extract_plan(T,E,opt_path);
path_length = opt_plan.nnodes;

t = [];
x = [];
time_offset = 0;

for ii=2:path_length
    t_ii = time_offset + opt_plan.Node{ii}.time;
    x_ii = opt_plan.Node{ii}.x;
    t = [t, t_ii];
    time_offset = t(end);
    x = [x x_ii];
end

x(5,isnan(x(5,:))>0)=0;
plot3(x(1,:),x(2,:),x(5,:),'k-d','linewidth',4);
movie_anima(frames) = getframe(figure(fig_xy));
frames = frames +1;
movie_anima(frames) = getframe(figure(fig_xy));
frames = frames +1;
movie_anima(frames) = getframe(figure(fig_xy));
frames = frames +1;
return
disp('saving video...');

for iter=1:frames-1
       vidObj.writeVideo(movie_anima(iter));
end
vidObj.writeVideo(movie_anima(iter));
   
close(vidObj);
   
disp('...done!');

close(fig_xy)