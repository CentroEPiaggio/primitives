function printafigu(path,name)
if ~exist(path,'dir')
    mkdir(path);
end

fname = [path name];
print('-dpng',[fname '.png']);
save([fname '.fig']);