function build_model( mdlName , len_ic)
% Make sure the current directory is writable because this example will be
% creating files.

[~, fa] = fileattrib(pwd);
if ~fa.UserWrite
    disp('This script must be run in a writable directory');
    return;
end

% Set the Inline parameters option and specify as Tunable the variables of
% interest (q0, qp0 in our case)
set_param(mdlName,'RTWInlineParameters','on');
set_param(mdlName,'TunableVars','q0,qp0');
set_param(mdlName,'TunableVarsStorageClass','Auto,Auto');
set_param(mdlName,'TunableVarsTypeQualifier',',');

% Build the RSim executable for the model. During the build process, a
evalin('base','q0 = [0,0,1];')
evalin('base','qp0 = [0,0,0];')
disp('Building compiled RSim simulation...')
rtwbuild(mdlName);
disp('Built RSim simulation')

% Get the Default Parameter Set for the Model
rtp = rsimgetrtp(mdlName,'AddTunableParamInfo','on');

for i = 1:len_ic
    rtp.parameters.values(1) = rtp.parameters.values(1) + 0.1;
    rtp.parameters.values(2) = rtp.parameters.values(1) + 0.1;
    rtp.parameters.values(3) = rtp.parameters.values(1) + 0.1;
    savestr = strcat('save params',num2str(i),'.mat rtp');
    eval(savestr);
end

end

