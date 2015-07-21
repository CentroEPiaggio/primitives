classdef Forward
    properties
        Control u; % generic control law
        Trig xi; % trigger conditions
        Duration t; % duration of execution of the primitive
        Params q; % task-specific parameters
        Imagespace chi; % image space of the primitive: see Planning_notes.tex for insight.
                        % These should be treated with a suitable (and easy to use) library
                        % as YALMIP.
        PrimitiveFun f; % The function that maps the params q to the imagespace chi.
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
            else
                fwd.u = [];
                fwd.xi = [];
                fwd.t = [];
                fwt.q = [];
                fwd.chi = [];
                fwd.f = PrimitiveFun; % instantiate a primitive function
            end
        end
        function evalprimitive(obj,q)
            res = f.eval(q);
            disp(num2str(res));
        end
    end
end