classdef Turn
    properties
        Control u;
        Trig xi;
        Duration t;
        Params q;
        Imagespace chi;
        PrimitiveFun f;
    end
    methods
        function fwd = Turn(u,xi,t,q,chi,f)
            if nargin>0
                fwd.u = u;
                fwd.xi = xi;
                fwd.t = t;
                fwd.q = q;
                fwd.chi = chi;
                fwd.f = f;
            end
        end
    end
end