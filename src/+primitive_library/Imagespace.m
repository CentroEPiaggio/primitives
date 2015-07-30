classdef Imagespace % this could hinerit Polyhedron but for now it doesn't
    properties
        P; % Polyhedron defining the space
        Pouter; % outer approximation of P
    end
    methods
        function obj = Imagespace(V)
            if nargin == 1 % specify vertices of polyhedron
                obj.P = Polyhedron(V);
                obj.P.computeHRep;
                obj.P.computeVRep;
                if ~obj.P.irredundantHRep
                    obj.P.minHRep;
                end
                if ~obj.P.irredundantVRep
                    obj.P.minVRep;
                end
            end
            
            % compute outerApproximation of the Polyhedron, useful to
            % speed_up sampling.
            % TODO: check/prove if formally correct
            obj.Pouter = obj.P.outerApprox;
        end
        function random_number = sample(obj)
            % TODO: insert bounds on space dimension
            % TODO: space should include a description in terms of:
            %       - its dimension
            %       - its limits (as above)
            %       - anything else?
            random_number = rand(obj.P.Dim,1);
            
            % sample inside a Polyhedron
            len = length(obj.P);
            % if the Polyhedron is a union of Polyhedra, choose randomly one of them to
            % sample within
            if len>1
                idx = randi(len);
            else
                idx = 1;
            end
            
            % Gaussian sampling inside a Polyhedron
            % random_number = obj.P(idx).randomPoint;
            % Uniform sampling inside a Polyhedron
            random_number = cprnd(1,obj.P.A,obj.P.b);
            random_number = random_number(:); % column vector
        end
    end
end