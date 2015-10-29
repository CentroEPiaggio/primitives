classdef Abbassa < Astratta
    properties
    end
    methods
        function obj=Abbassa(q)
            obj.q = q;
        end
        function obj = steering(q)
            disp('Abbassa');
            disp('meh')
            q
            disp('boh')
        end
    end
end