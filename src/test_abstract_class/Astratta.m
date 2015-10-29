classdef Astratta
    properties
        q;
    end
    methods
        function obj=Astratta(obj,q)
            disp('costruttor Astratta');
            keyboard
            obj.q = q;
        end
        function printa_q(obj)
            obj.q
        end
    end
    methods (Abstract)
        steering(obj)
        % this function is the steering function for the primitive
    end
end