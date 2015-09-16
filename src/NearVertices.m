function idX_near = NearVertices(points_mat, x_new, n)
% returns the Nearest node set to the x_rand node, depending on n parameter
idX_near = rangesearch(points_mat',x_new',n);


end

