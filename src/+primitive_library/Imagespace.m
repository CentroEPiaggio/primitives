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
    end
end