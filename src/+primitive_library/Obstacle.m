classdef Obstacle % this could hinerit Polyhedron but for now it doesn't
    properties
        P; % Polyhedron defining the space
        Pouter; % outer approximation of P
        dimensions; % dimensions where the trajectories should be checked for collisions (e.g. if the trajectory lies in [x,y,theta,v] for the differential drive vehicle, we might want to test for obstacles only in the [x,y] dimensions.)
    end
    methods
        function obj = Imagespace(V,dimensions)
            if nargin >= 1 % specify vertices of polyhedron
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
            
            if nargin == 2
                obj.dimensions = dimensions;
            end
            % compute outerApproximation of the Polyhedron, useful to
            % speed_up sampling.
            % TODO: check/prove if formally correct
            obj.Pouter = obj.P.outerApprox;
        end
    end
end