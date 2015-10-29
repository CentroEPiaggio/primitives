fuck = cell(1);
kk=1;
for ii=1:size(E,1)
    for jj=1:size(E,2)
        if ~isempty(E{ii,jj})
            fuck{kk} = E{ii,jj};
            kk=kk+1;
        end
    end
end

%%
for ii=1:length(fuck)
    if size(fuck{ii}.x,1) == 1
        disp('fuck')
        keyboard
    end
end