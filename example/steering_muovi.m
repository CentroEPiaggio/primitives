% STEERING_MUOVI
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
function [feasible,cost,q,traj_pos_cart,traj_vel_cart]=steering_muovi(xi,xf,vi,vf)
% initialization
feasible=0;
cost=Inf;
q=[xi,xf,vi,vf];
traj_pos_cart=NaN;
traj_vel_cart=NaN;

run_filepath = '../example/';
prim_filepath = [run_filepath 'prim/'];


Tend = 10; % TODO porcata. Il tempo va parametrizzato.
Ts = 0.01;

enable_muovi = true;
enable_alza  = false;
if enable_muovi==false && enable_alza==false % what's your game?'
    feasible = false;
    cost = Inf;
    return;
end

% xi = 0;
% xf = 10;
yi = rand;
yf = yi*2;
% prepare data for muovi
if enable_muovi
    % define parameters for primitive muovi
    primitive_muovi_params = struct('name','muovi',    ...
        'xi',xi,            ...
        'xf',xf,            ...
        'vi',vi, ...
        'vf',vf, ...
        'Tend',Tend,        ...
        'Ts',Ts,            ...
        'xf_vec_len',1, ...
        'vx0_vec_len',1,  ...
        'vxf_vec_len',1, ...
        'filepath',prim_filepath ...
        );
%     [time,traj_x_cart]=gen_primitives_muovi(primitive_muovi_params);
    [time,traj_vel_cart,q]=gen_primitives_muovi_local(primitive_muovi_params);
    if any(isnan(time)) || any(isnan(traj_vel_cart))
        feasible=0;
        cost=Inf;
        return
    end
    if nargout > 2 % if requested...
        traj_pos_cart = xi+cumtrapz(time,traj_vel_cart); % ...returns both the position trajectory...
        traj_vel_cart = traj_vel_cart;                % ...and the velocity trajectory
        q = [xi traj_pos_cart(end) vi traj_vel_cart(end)];
    end
%     keyboard
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
    traj_y_cart = zeros(size(traj_vel_cart));
end
% generate initial condition file for simulation
q0 = [xi(:);deg2rad(90);2];
qp0 = [vi(:);0;0];
qref0 = q0;
ic = struct('q0',q0, 'qp0',qp0, 'qref0',qref0);
gen_ic(ic);
% simulate
q_reference = [time(:)';
    traj_vel_cart(:)';
    traj_y_cart(:)'];
save('../example/runna.mat','q_reference');
runstr = [run_filepath, 'modello -f rsim_tfdata.mat=' run_filepath 'runna.mat -p ' run_filepath 'params_steering.mat -v -tf ',num2str(Tend)];
[status, result] = system(runstr);
if status ~= 0, error(result); end
% check if feasible
load('../src/modello.mat');
% disp(num2str(rt_zmpflag(end))) % display feasibility bit
if rt_zmpflag(end)==0
    feasible = 1;
    cost = rt_cost(end);
else
    feasible = 0;
    cost = Inf;
end
% pack return data