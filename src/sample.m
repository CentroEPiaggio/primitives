function random_number = sample(space)
% TODO: insert bounds on space dimension
% TODO: space should include a description in terms of:
%       - its dimension
%       - its limits (as above)
%       - anything else?
random_number = rand(space.P.Dim,1);

% sample inside a Polyhedron
len = length(space.P);
% if the Polyhedron is a union of Polyhedra, choose randomly one of them to
% sample within
if len>1
    idx = randi(len);
else
    idx = 1;
end

% Gaussian sampling inside a Polyhedron
% random_number = space.P(idx).randomPoint;
% Uniform sampling inside a Polyhedron
random_number = cprnd(1,space.P.A,space.P.b);
random_number = random_number(:); % column vector
end