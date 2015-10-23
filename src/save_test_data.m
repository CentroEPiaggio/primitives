%% save test data
if ~exist('test/','dir')
    mkdir('test/');
end
custom_test_name = []; % use custom_test_name = 'mytest' to firce filename and avoid timestamp;
if isempty(custom_test_name)
    formatOut = 'yyyy_mm_dd_hh_MM_SS';
    str_date=datestr(now,formatOut);
    test_savestr = ['test/test_' str_date '.mat'];
else
    test_savestr = ['test/test_' num2str(custom_test_name) '.mat'];
end
save(test_savestr); % run load(test_savestr) to reload this data
