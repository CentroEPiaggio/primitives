classdef Muovi < Astratta
    properties
    end
    methods
        function obj=Muovi(q)
            obj.q = q;
        end
        function obj = steering(q)
            disp('Muovi');
            disp('beh')
            q
            disp('top')
        end
    end
end