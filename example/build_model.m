function build_model( mdlName , len_ic, params)
% HARDFIX for fixing compilation error when the file rsim_tfdata.mat does
% not exists yet. Also checking if a file exists with esist(filename)
% returns false positives... bad implementation on mathworks side?
if ~any(findstr(ls,'rsim_tfdata.mat'))
    % to compute the checksum of a simulink model it needs the
    % rsim_tfdata.mat file to exist.
    q_reference=zeros(3,10);
    save('rsim_tfdata.mat','q_reference');
%     system('cp prim/primitiva_muovi_1_1_1_1.mat rsim_tfdata.mat');
end

% Open the model and configure it to use the RSim target.
open_system(mdlName);
cs = getActiveConfigSet(mdlName);
% Switch target to rsim
cs.switchTarget('rsim.tlc',[]);

% %% Checking if the current version of the model has been already compiled
% % using chesksums
% checksum1=Simulink.BlockDiagram.getChecksum(mdlName);
% if exist('checksum.mat','file')==2
%     % tricky stuff
%     temp=checksum1;
%     load('checksum.mat');
%     checksum2=checksum1;
%     checksum1=temp;
%     
%     if (checksum1 ~= checksum2)
%         disp(' -- Checksums are different: rebuild! -- ')
%         save('checksum.mat','cs1');
% %         build_model(mdlName,len_ic);
%     else
%         disp(' -- Checksums are the same: thou shall not build! -- ')
%         return
%     end
% else
%     disp(' -- Checksum never computed: creating file! -- ')
%     save('checksum.mat','checksum1');
% %     build_model(mdlName, len_ic);
% end
% 
%% Make sure the current directory is writable because this example will be
% creating files.
[~, fa] = fileattrib(pwd);
if ~fa.UserWrite
    disp('This script must be run in a writable directory');
    return;
end

% Set the Inline parameters option and specify as Tunable the variables of
% interest (q0, qp0 in our case)
set_param(mdlName,'RTWInlineParameters','on');
set_param(mdlName,'TunableVars','q0,qp0,qref0');
set_param(mdlName,'TunableVarsStorageClass','Auto,Auto,Auto');
set_param(mdlName,'TunableVarsTypeQualifier',',,');

% Build the RSim executable for the model. During the build process, a
% evalin('base','q0 = [0,0,1];')

evalin('base',['q0 = [' num2str(params.q0(:)') '];'])
evalin('base',['qp0 = [' num2str(params.qp0(:)') '];'])
evalin('base',['qref0 = [' num2str(params.qref0(:)') '];'])
disp('Building compiled RSim simulation...')
rtwbuild(mdlName);
disp('Built RSim simulation')

%% Get the Default Parameter Set for the Model
% keyboard
rtp = rsimgetrtp(mdlName,'AddTunableParamInfo','on');

n_indep_coords = length(params.q0);
rtp.parameters.values(1:n_indep_coords) = params.q0(:)';
rtp.parameters.values(n_indep_coords+1:2*n_indep_coords) = params.qp0(:)';
rtp.parameters.values(2*n_indep_coords+1:3*n_indep_coords) = params.qref0(:)';

% savestr = strcat('save params',num2str(i),'.mat rtp');
% eval(savestr);
increment = 0.1;
for i = 1:len_ic
%     rtp.parameters.values(1) = rtp.parameters.values(1) + 0.1;
%     rtp.parameters.values(2) = rtp.parameters.values(1) + 0.1;
%     rtp.parameters.values(3) = rtp.parameters.values(1) + 0.1;
    rtp.parameters.values(1:n_indep_coords) = rtp.parameters.values(1:n_indep_coords) + increment*(i-1);
    rtp.parameters.values(n_indep_coords+1:2*n_indep_coords) = rtp.parameters.values(n_indep_coords+1:2*n_indep_coords) + increment*(i-1);
    rtp.parameters.values(2*n_indep_coords+1:3*n_indep_coords) = rtp.parameters.values(2*n_indep_coords+1:3*n_indep_coords) + increment*(i-1);
    savestr = strcat('save params',num2str(i),'.mat rtp');
    eval(savestr);
end
%% Using RSim Target for Batch Simulations
% The MAT-file rsim_tfdata.mat is required in the local directory.
if ~isempty(dir('rsim_tfdata.mat')),
    delete('rsim_tfdata.mat');
end
tudata = zeros(3,100);
save('rsim_tfdata.mat','tudata')

end

