function make_ic(mdlName,len_ic)
%% Get the Default Parameter Set for the Model
disp('Generating initial conditions... DONE');
rtp = rsimgetrtp(mdlName,'AddTunableParamInfo','on');
% rtp.modelChecksum = cast(rtp.modelChecksum,'uint32'); %HARDFIX
%%
i=1;
savestr = strcat('save params',num2str(i),'.mat rtp');
eval(savestr);
% for i = 1:len_ic
%     rtp.parameters.values(1) = rtp.parameters.values(1) + 0.1;
%     rtp.parameters.values(2) = rtp.parameters.values(1) + 0.1;
%     rtp.parameters.values(3) = rtp.parameters.values(1) + 0.1;
%     savestr = strcat('save params',num2str(i),'.mat rtp');
%     eval(savestr);
% end
disp('Generating initial conditions... DONE');