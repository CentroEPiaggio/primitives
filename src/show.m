movie = 1;
%% go to test the plan
% assemble the optimal plan
Tend = 10; % TODO porcata. Il tempo va parametrizzato.
Ts = 0.01;
run_filepath = '../example/';
prim_filepath = [run_filepath 'prim/'];
% init
q_reference = [0;0;0];
% loop
for ii=2:length(opt_plan.Node)
    time = opt_plan.Node{ii}.time;
    traj_x_speed_cart = opt_plan.Node{ii}.x(2,:);
    traj_y_speed_cart = gradient(opt_plan.Node{ii}.x(3,:))/mean(diff(opt_plan.Node{ii}.time));
    q_reference_add = [q_reference(1,end)+time(:)';
        traj_x_speed_cart(:)';
        traj_y_speed_cart(:)'];
    q_reference = [q_reference, q_reference_add];
end

if(movie==1)
    disp('saving rrtstar video...');
    
    for iter=1:frames-1
        vidObj.writeVideo(movie_rrtstar(iter).cdata(:,:,:));
    end
    
    close(vidObj);
    
    disp('...done!');
end

save([run_filepath 'runna.mat'],'q_reference');
save([run_filepath 'rsim_tfdata.mat'],'q_reference');
Tend = q_reference(1,end)*1.1; % 10 percent more time, for the show
q0 = [0;deg2rad(90);2];
qp0 = [0;0;0];
qref0 = q0;
ic = struct('q0',q0,'qp0',qp0,'qref0',qref0);
gen_ic(ic);
m1 = 100;
m2 = 1;
m3 = 100;
masses=[m1;m2;m3];
nq=length(q0);
runstr = [run_filepath, 'modello -f rsim_tfdata.mat=' run_filepath 'runna.mat -p ' run_filepath 'params_steering.mat -o ' run_filepath 'optimal.mat -v -tf ',num2str(Tend)];
[status, result] = system(runstr);
if status ~= 0, error(result); end
% and the show must go on!
load ../example/optimal.mat;
figure
plot(rt_t,rt_qp_ref,rt_t,rt_qp,q_reference(1,:),q_reference(2,:))
grid on
anima