classdef PrimitiveFun
    properties
        chi; % Imagespace
%         Params q;
%         Mapping f;
    end
    methods
        % constructor
        function obj = PrimitiveFun(V) % TODO: PrimitiveFun(chi,q,f)
import primitive_library.*;
            if nargin == 1
                obj.chi = Imagespace(V);
            elseif nargin < 1
                obj.chi = Imagespace([-1 -1; -1 1; 1 -1; 1 1]*0.3);
            end
        end
        function eval(obj,q)
            % task-specific function that maps the parameter space to the
            % image space
            % here should go abstract functions over primitives
            chi = 2*q;
        end
    end
end