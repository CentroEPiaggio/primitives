% assemble the optimal plan
Ts = 0.01;
run_filepath = '../example/';
prim_filepath = [run_filepath 'prim/'];
% init
q_reference = [0;0;0];
% loop
for kk=2:opt_plan.nnodes
    if debug,keyboard,end
    time = opt_plan.Node{kk}.time;
    traj_x_speed_cart = opt_plan.Node{kk}.x(2,:);
    traj_y_speed_cart = gradient(opt_plan.Node{kk}.x(3,:))/mean(diff(opt_plan.Node{kk}.time));
%     if strcmp(opt_plan.Node{kk-1}.primitive,'Eleva') && strcmp(opt_plan.Node{kk}.primitive,'Muovi')
%         disp('Eleva after Muovi. What''s going on?')
%         keyboard
%     end
    q_reference_add = [q_reference(1,end)+time(:)'; % conventionally SIMULINK requires that the first row of the vector q_reference (used in a from_file block) is the time
        traj_x_speed_cart(:)';
        traj_y_speed_cart(:)'];
    q_reference = [q_reference, q_reference_add];
    kk
%     keyboard
end
% return
Tend = q_reference(1,end); % DONE. Il tempo della simulazione ora e' parametrizzato.
movie=1;
if(movie==1)
    disp('saving rrtstar video...');
    
    for iter=1:frames-1
        vidObj.writeVideo(movie_rrtstar(iter).cdata(:,:,:));
    end
    
    close(vidObj);
    
    disp('...done!');
end

save([run_filepath 'runna.mat'],'q_reference');
% save([run_filepath 'rsim_tfdata.mat'],'q_reference');
% save([run_filepath 'params_steering.mat'],'q_reference');
Tend = q_reference(1,end)*1.1; % 10 percent more time, for the show
q0 = [0;deg2rad(90);2];
q0 = [opt_plan.Node{2}.x(1,1); deg2rad(90); opt_plan.Node{2}.x(1,3)];
% q0 = opt_plan.Node{2}.x(:,1);
qp0 = [opt_plan.Node{2}.x(2,1);0;0];
qref0 = q0;
ic = struct('q0',q0,'qp0',qp0,'qref0',qref0);
gen_ic(ic);
% m1 = 100;
% m2 = 1;
% m3 = 100;
% masses=[m1;m2;m3];
nq=length(q0);
runstr = [run_filepath, 'modello -f rsim_tfdata.mat=' run_filepath 'runna.mat -p ' run_filepath 'params_steering.mat -o ' run_filepath 'optimal.mat -v -tf ',num2str(Tend)];
[status, result] = system(runstr);
if status ~= 0, error(result); end
% and the show must go on!
load ../example/optimal.mat;
%%
figure
plot(rt_t,rt_qp_ref,'--','Linewidth',2)
hold on
plot(rt_t,rt_qp,'Linewidth',2)
grid on
title('Speed profile')
figure
plot(rt_t,rt_q_ref,'--','Linewidth',2)
hold on
plot(rt_t,rt_q,'Linewidth',2)
grid on
title('Position profile')
%%
anima