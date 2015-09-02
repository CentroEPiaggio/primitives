function load_model( mdlName )
% Make sure the current directory is writable because this example will be
% creating files.

[stat, fa] = fileattrib(pwd);
if ~fa.UserWrite
    disp('This script must be run in a writable directory');
    return;
end

% Open the model and configure it to use the RSim target.
open_system(mdlName);
cs = getActiveConfigSet(mdlName);
cs.switchTarget('rsim.tlc',[]);

% Set the Inline parameters option and specify as Tunable the variables of
% interest (q0, qp0 in our case)
set_param(mdlName,'RTWInlineParameters','on');
set_param(mdlName,'TunableVars','q0,qp0');
set_param(mdlName,'TunableVarsStorageClass','Auto,Auto');
set_param(mdlName,'TunableVarsTypeQualifier',',');

% The MAT-file rsim_tfdata.mat is required in the local directory.
if ~isempty(dir('rsim_tfdata.mat')),
    delete('rsim_tfdata.mat');
end
tudata = zeros(3,100);
save('rsim_tfdata.mat','tudata')

% Build the RSim executable for the model. During the build process, a
evalin('base','q0 = [0,0,1];')
evalin('base','qp0 = [0,0,0];')
disp('Building compiled RSim simulation...')
rtwbuild(mdlName);
disp('Built RSim simulation')

% Get the Default Parameter Set for the Model
rtp = rsimgetrtp(mdlName,'AddTunableParamInfo','on');

for i = 1:10
    rtp.parameters.values(1) = rtp.parameters.values(1) + 0.1;
    rtp.parameters.values(2) = rtp.parameters.values(1) + 0.1;
    rtp.parameters.values(3) = rtp.parameters.values(1) + 0.1;
    savestr = strcat('save params',num2str(i),'.mat rtp');
    eval(savestr);
end

% disp('Creating rtP data files...')
% for i=1:10
%   % Extract current rtP structure using new damping factor.
%   [rtpstruct]=evalin('base','rsimgetrtp(''rtwdemo_rsimtf'');');
%   savestr = strcat('save params',num2str(i),'.mat rtpstruct');
%   eval(savestr);
%   evalin('base','theta = theta - .1;');
% end
% disp('Finished creating parameter data files.')
end

