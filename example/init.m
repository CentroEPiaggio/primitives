clear all; close all; clc;

nq = 3;
q0 = zeros(nq,1);
q0(3) = 0.1; % altrimenti diventa singolare
qp0 = zeros(nq,1);
q0 = rand(nq,1)
q0(2)=0;
q0(3)=1;
% qp0 = rand(nq,1)
% q0 = [1,deg2rad(45),2];

% parameters
m1 = 100;
m2 = 1;
m3 = 100;
masses=[m1;m2;m3];

% sampling time for simulation (discrete time)
Ts = 0.001;

% dimensioni carrello (per anima.m e check ZMP)
cart_width  = 2;
cart_height = 0.4;

% traiettorie carrello
xi=q0(1);
xf=1;
xpi=0;
xpf=0;

yi=0;
ypi=0;
ypf=0;
T = [0 1];
[time,traj_x_cart] = muovi(xi,xf,xpi,xpf,T(end),Ts);
% correct format for .mat loading:
% [t(1) t(2) ... t(N);
% [y(1) y(2) ... y(N)];
q_reference = [time(:)';traj_x_cart(:)'];
save primitiva_muovi.mat q_reference;
% simulation total time
Tend = T(end)*10;

%% Using RSim Target for Batch Simulations

%%
% Make sure the current directory is writable because this example will be
% creating files.
[stat, fa] = fileattrib(pwd);
if ~fa.UserWrite
    disp('This script must be run in a writable directory');
    return;
end

%%
% Open the model and configure it to use the RSim target.
mdlName = 'modello';
open_system(mdlName);
cs = getActiveConfigSet(mdlName);
cs.switchTarget('rsim.tlc',[]);

%%
% The MAT-file rsim_tfdata.mat is required in the local directory.
if ~isempty(dir('rsim_tfdata.mat')),
    delete('rsim_tfdata.mat');
end
str1 = fullfile(matlabroot,'toolbox','rtw','rtwdemos','rsimdemos','rsim_tfdata.mat');
str2 = ['copyfile(''', str1, ''',''rsim_tfdata.mat'',''writable'')'];
eval(str2);

%%
% Build the RSim executable for the model. During the build process, a
disp('Building compiled RSim simulation.')
rtwbuild(mdlName);
%%
% Generate Primitive .mat files
prim_path = 'prim/';
if ~exist(prim_path,'dir')
    mkdir(prim_path)
end

% Now using together x and y trajectories
xf_vec = linspace(-10,10,10);
yf_vec = linspace(0,10,10);
disp('Generating primitives...');
for k=1:length(yf_vec)
    yf = yf_vec(k);
    [time,traj_y_cart] = muovi(yi,yf,ypi,ypf,T(end),Ts);
    for i=1:length(xf_vec)
        xf = xf_vec(i);
        [time,traj_x_cart] = muovi(xi,xf,xpi,xpf,T(end),Ts);
        q_reference = [time(:)';traj_x_cart(:)';traj_y_cart(:)'];
        savestr = strcat('save primitiva_muovi_',num2str(i),'_',num2str(k),'.mat q_reference');
        eval(savestr);
    end
end
disp('Generating primitives... DONE');

%% Run the RSim Compiled Simulation Using New Signal Data

disp('Starting batch simulations.')

tic
for k=1:length(yf_vec)
    for i = 1:length(xf_vec)
      % Bang out and run the next set of data with RSim
      runstr = ['.', filesep, 'modello -f rsim_tfdata.mat=primitiva_muovi_', ...
                num2str(i),'_',num2str(k),'.mat -v -tf 5.000'];
      [status, result] = system(runstr);
      if status ~= 0, error(result); end
      % Load the data to MATLAB and plot the results.
      load modello.mat
      plot(rt_q); % now for example the q of the system are plotted
      grid on
      hold on
    end
end
c=toc;
disp(strcat('Average computation time',{': '},num2str(c/length(xf_vec))));

return 

%% OLD VERSION

prim_path = 'prim/';
if ~exist(prim_path,'dir')
    mkdir(prim_path)
end
xf_vec = linspace(-10,10,10);
disp('Generating primitives...');
for ii=1:length(xf_vec)
    xf = xf_vec(ii);
    [time,traj_x_cart] = muovi(xi,xf,xpi,xpf,T(end),Ts);
    q_reference = [time(:)';traj_x_cart(:)'];
    save([prim_path 'primitiva_muovi_' num2str(ii) '.mat'],'q_reference');
end
disp('Generating primitives... DONE');

%% loops!
% first loop: between every 
% possible reference value
% for the primitive MUOVI
out_path = 'output/';
if ~exist(out_path,'dir')
    mkdir(out_path)
end
disp(['Starting simulation loop for muovi: ' datestr(now)]);
for ii=1:length(xf_vec)
    disp(num2str(ii));
    % use variable primitiva_muovi to load the .mat file to be used within the
    % current simulation
    primitiva_muovi = [prim_path 'primitiva_muovi_' num2str(ii) '.mat'];
    % (builds and) simulates the simulink file
    tic
    sim('modello')
    toc
    % save output. The compiled version does it automatically
    output = [t(:)';q(:,1)'];
    save([out_path 'out_primitiva_muovi_' num2str(ii) '.mat'],'output');
end
disp(['Starting simulation loop for muovi: DONE at time ' datestr(now)]);

%% plot output
figure
for ii=1:length(xf_vec)
    clear output;
    disp(num2str(ii));
    load([out_path 'out_primitiva_muovi_' num2str(ii) '.mat']);
    time = output(1,:);
    x = output(2,:);
    plot(time,x);
    hold on
end
xlabel('time [s]')
title('cart position')
return
%%
% tic
% !./modello
% toc
%% load('modello.mat');
% warning('off')
% tic
% sim('modello')
% toc
plottini