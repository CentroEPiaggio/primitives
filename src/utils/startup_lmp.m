% startup_lmp: startup LocoManiPlanner script
if strfind(path,'cprnd') % very dirt workaround trick. Checks if the last addpath has been executed in the current session.
    return
end

%% here go initialization commands
cd ../../tbxmanager
startup % run startup script for mpt_toolbox libary
cd ../src

cd ../example/
addpath(pwd)
cd ../src
addpath(pwd) % add current dir to path
addpath('cprnd/'); % add uniform sampler library to path

