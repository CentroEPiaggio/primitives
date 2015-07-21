classdef PrimitiveFun
    properties
        Params q;
        Imagespace chi;
    end
    methods
        function eval(obj,q)
            % task-specific function that maps the parameter space to the
            % image space
            % here should go abstract functions over primitives
            chi = 2*q;
        end
    end
end