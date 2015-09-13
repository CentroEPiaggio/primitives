% GEN_IC
% generate initial condition file for simulating inside a steering function
function gen_ic(params)
% rtp = rsimgetrtp(mdlName,'AddTunableParamInfo','on');
ic=load('params1.mat');
n_indep_coords = length(params.q0);
ic.rtp.parameters.values(1:n_indep_coords) = params.q0(:)';
ic.rtp.parameters.values(n_indep_coords+1:2*n_indep_coords) = params.qp0(:)';
ic.rtp.parameters.values(2*n_indep_coords+1:3*n_indep_coords) = params.qref0(:)';
rtp = ic.rtp;
savestr = strcat('save ../example/params_steering.mat rtp');
eval(savestr);