% STEERING
% this function takes as arguments the following:
% enable_muovi: true or false
% enable_alza:  true or false
% xi: horizontal initial position
% xf: horizontal final position
% yi: vertical initial position
% yf: vertical final position
% the function returns the following:
% feasible: true or false
% cost: the cost of performing such action
% data: the data to be added to the plan tree?
prim_filepath = 'prim/';

enable_muovi = true;
enable_alza  = true;
if enable_muovi==false && enable_alza==false % what's your game?
    return;
end

xi = 0;
xf = 10;
yi = rand;
yf = yi*2;
% prepare data for muovi
if enable_muovi
    % define parameters for primitive muovi
    primitive_muovi_params = struct('name','muovi',    ...
        'xi',xi,            ...
        'xf',xf,            ...
        'Tend',Tend,        ...
        'Ts',Ts,            ...
        'xf_vec_len',1, ...
        'vx0_vec_len',1,  ...
        'vxf_vec_len',1, ...
        'filepath',prim_filepath ...
        );
    [time,traj_x_cart]=gen_primitives_muovi(primitive_muovi_params);
end

% prepare data for alza
if enable_alza
    primitive_abbassa_params = struct('name','abbassa',    ...
        'yi',yi,            ...
        'yf',yf, ...
        'ypi',0,            ...
        'ypf',0,          ...
        'Tend',Tend,        ...
        'Ts',Ts,            ...
        'yf_vec_len',1,    ...
        'filepath',prim_filepath ...
        );
    [time,traj_y_cart]=gen_primitives_abbassa(primitive_abbassa_params);
else
    traj_y_cart = zeros(size(traj_x_cart));
end
% simulate
q_reference = [time(:)';
    traj_x_cart(:)';
    traj_y_cart(:)'];
save('runna.mat','q_reference');
runstr = ['.', filesep, 'modello -f rsim_tfdata.mat=runna.mat -p params',num2str(i),'.mat -v -tf ',num2str(Tend)];
[status, result] = system(runstr);
if status ~= 0, error(result); end
% check if feasible
load('modello.mat');
disp(num2str(rt_zmpflag(end)))
% pack return data