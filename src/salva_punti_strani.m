points_mat = cell2mat(T.Node');
    % now since we want to search in Chi0 we can remove all rows from
    % points_mat that does contain a NaN
    %     points_mat(~any(isnan(points_mat)),:)=[]
    points_mat(isnan(points_mat)) = []; % remove NaN from points
    points_mat = reshape(points_mat,2,size(T.Node,1))
    PUNTI_FINTI=points_mat(:,2:end);
    save prova_punti_strani.mat PUNTI_FINTI;