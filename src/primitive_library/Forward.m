classdef Forward
    properties
        Control u;
        Trig xi;
        Duration t;
        Params q; % task-specific
        Imagespace chi; % da trattare con Yalmip
        PrimitiveFun f; % per trovare il sistema finale
    end
    methods
        function fwd = Forward(u,xi,t,q,chi,f)
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