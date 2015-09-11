clear all; close all; clc;

nq = 3;
q0 = zeros(nq,1);
q0(3) = 0.1; % altrimenti diventa singolare
qp0 = zeros(nq,1);
% qp0 = rand(nq,1);
q0 = rand(nq,1)
% q0(2)=0;
% q0(3)=1;
% qp0 = rand(nq,1)
q0 = [0,deg2rad(90),1];
q0 = [0,deg2rad(10),2];
qref0=q0;

len_ic = 1;
prim_filepath='prim/';

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
xf=1; xf=.1;
xpi=0;
xpf=0;

yi=0;
ypi=0;
ypf=0;
T = [0 1];
%[time,traj_x_cart] = muovi(xi,xf,xpi,xpf,T(end),Ts);
% correct format for .mat loading:
% [t(1) t(2) ... t(N);
% [y(1) y(2) ... y(N)];
%q_reference = [time(:)';traj_x_cart(:)'];
%save primitiva_muovi.mat q_reference;
% simulation total time

% Tend = T(end)*10;
Tend = T(end)*10;

%% build the model
% setup ic structure for building
ic = struct('q0',q0, 'qp0',qp0, 'qref0',qref0);
% build
build_model( 'modello' , len_ic, ic)

%% Generate files with primitive trajectories to be used as input signals
howfuckedweare = 1; % the larger this number the slower the generation of the primitive image spaces.
% Since we have nested for loop the time grows with
% howfuckedweare^n
xf_vec_len = howfuckedweare;     % how many points to sample for xf?
yf_vec_len = howfuckedweare;     % how many points to sample for yf?
vx0_vec_len = howfuckedweare;    % how many points to sample for vx0?
vxf_vec_len = howfuckedweare;    % how many points to sample for vxf?

% define parameters for primitive muovi
primitive_muovi_params = struct('name','muovi',    ...
    'xi',xi,            ...
    'xf',xf,            ...
    'Tend',Tend,        ...
    'Ts',Ts,            ...
    'xf_vec_len',xf_vec_len, ...
    'vx0_vec_len',vx0_vec_len,  ...
    'vxf_vec_len',vxf_vec_len, ...
    'filepath',prim_filepath ...
    )
primitive_abbassa_params = struct('name','abbassa',    ...
    'yi',yi,            ...
    'ypi',ypi,            ...
    'ypf',ypf,          ...
    'Tend',Tend,        ...
    'Ts',Ts,            ...
    'yf_vec_len',yf_vec_len,    ...
    'filepath',prim_filepath ...
    )
make_primitives_muovi(primitive_muovi_params)
make_primitives_abbassa(primitive_abbassa_params)

%% Run the RSim Compiled Simulation Using New Signal Data
close all
disp('Starting batch simulations.')

% trick for speeding up the parfor loops: define variables as local, hence
% not using a shared structure.
y_len=primitive_abbassa_params.yf_vec_len;
xf_len=primitive_muovi_params.xf_vec_len;
vx0_len=primitive_muovi_params.vx0_vec_len;
vxf_len=primitive_muovi_params.vxf_vec_len;


% main for loop
tic
for i =1:len_ic
    for y=1:y_len
        for x = 1:xf_len
            for vi = 1:vx0_len
                for vf = 1:vxf_len
                    % Bang out and run the next set of data with RSim
%                     the next 2 lines if in a single file there are both
%                     primitives stored together. After those two lines we
%                     build a new file with the primitives taken separately
%                     and merged together
%                     test_codenum = [num2str(x),'_',num2str(y),'_',num2str(vi),'_',num2str(vf)];
%                     primitiva_muovi_loadstr = [prim_filepath, 'primitiva_muovi_',test_codenum,'.mat'];
test_codenum_muovi = [num2str(x),'_',num2str(vi),'_',num2str(vf)];
test_codenum_abbassa = [num2str(y)];
primitiva_muovi_loadstr = [prim_filepath, 'primitiva_muovi_',test_codenum_muovi,'.mat'];
primitiva_abbassa_loadstr = [prim_filepath, 'primitiva_abbassa_',test_codenum_abbassa,'.mat'];
muovi=load(primitiva_muovi_loadstr);
abbassa=load(primitiva_abbassa_loadstr);
q_reference = [muovi.q_reference(1,:);
    muovi.q_reference(2,:);
    abbassa.q_reference(2,:)];
save('runna.mat','q_reference');
                    runstr = ['.', filesep, 'modello -f rsim_tfdata.mat=runna.mat -p params',num2str(i),'.mat -o run_muovi_',test_codenum_muovi,'_',test_codenum_abbassa,'.mat -v -tf ',num2str(Tend)];
% runstr = ['.', filesep, 'modello -f rsim_tfdata.mat=',primitiva_muovi_loadstr,' -p params',num2str(i),'.mat -o run_muovi_',test_codenum,'.mat -v -tf 0.4'];
                    [status, result] = system(runstr);
                    if status ~= 0, error(result); end
                    % Load the data to MATLAB and plot the results.
                    %                                   rtdata=load('modello.mat');
                    %                                   plot(rtdata.rt_q);
                    %                hold on; grid on;
                    % save data in imagespace matrix
                    %                    if rt_zmpflag(end)==1       % simulation stopped due to ZMP violation
                    %                        imagespace(i,x,vf) = 0;    % exclude from imagespace
                    %                    else imagespace(i,x,vf) = 1;   % include into imagespace
                    %                    end
                end
            end
        end
    end
end
c=toc;
disp('Finished simulations.')
disp(['Total simulation time: ' num2str(c)])
disp(strcat('Average computation time',{': '},num2str(c/primitive_muovi_params.xf_vec_len)));

%% data post-processing
% test imagespace saving
% for MUOVI we sample in final position and velocity
imagespace = zeros(len_ic,y_len,xf_len,vx0_len,vxf_len);
% save data in imagespace matrix from all the runs
for i =1:len_ic
    for y=1:y_len
        for x = 1:xf_len
            for vi = 1:vx0_len
                for vf = 1:vxf_len
                    test_codenum = [num2str(x),'_',num2str(y),'_',num2str(vi),'_',num2str(vf)];
                    load(['run_muovi_' test_codenum '.mat']);
%                     keyboard
                    % save data in imagespace matrix
                    if rt_zmpflag(end)==1       % simulation stopped due to ZMP violation
                        imagespace(i,x,vf) = 0;    % exclude from imagespace
                    else imagespace(i,x,vf) = 1;   % include into imagespace
                    end
                end
            end
        end
    end
end
%%
figure
plotdata = imagespace(1,1,1,:,:);
plotdata = squeeze(plotdata);
imagesc(plotdata)
colormap([1 1 1; 0 0 0])
% set(gca,'XTickLabel',round(xf_vec));
% set(gca,'YTickLabel',round(vxf_vec));
% set(gca,'XTickLabel',{'-3pi','-2pi','-pi','0','pi','2pi','3pi'})
% set(gca,'YTickLabel',{'min = -1','-0.5','0','0.5','max = 1'})
cbr=colorbar;
set(cbr,'YTick',0:1)
grid on
xlabel('xf\_vec')
ylabel('vxf\_vec')
title('MUOVI primitive imagespace')
% save prim1.mat imagespace xf_vec yf_vec;
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
% plottini