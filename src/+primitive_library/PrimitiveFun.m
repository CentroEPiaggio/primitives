classdef PrimitiveFun
    properties
        name; % string
        chi; % Imagespace
%         Params q;
%         Mapping f;
cost_coeff;  % replace with an abstract function or sth else
cost_table;
    end
    methods
        % constructor
        function obj = PrimitiveFun(V,cost_coeff,cost_table,name) % TODO: PrimitiveFun(chi,q,f)
import primitive_library.*;
            if nargin == 4
                obj.chi = Imagespace(V);
                obj.cost_coeff = cost_coeff;
                obj.cost_table = cost_table;
                obj.name = name;
            elseif nargin < 1
                obj.chi = Imagespace([-1 -1; -1 1; 1 -1; 1 1]*0.3);
            end
        end
        function string = getName(obj)
            string = obj.name;
        end
        function eval(obj,q)
            % task-specific function that maps the parameter space to the
            % image space
            % here should go abstract functions over primitives
            chi = 2*q;
        end
        function c = cost(obj,q) % TODO: define better cost interface
            c = polyval(cost_coeff,q);
        end
        function q = findbest(obj,x_i,x_f)
            % recenter
            x_f = x_f-x_i;
            x_i = 0*x_i;
            path_to_xf = x_f;
%             path_to_x_f = norm(x_f);
            tol = 0.01;
            idx = find(abs(obj.cost_table(:,3)-path_to_x_f)<=tol);
            while isempty(idx) && tol < 0.2
                tol = 2*tol;
                idx = find(norm(obj.cost_table(:,3)-path_to_x_f)<=tol);
            end
            [~,idx_opt] = min(obj.cost_table(idx,1));
            q = obj.cost_table(idx(idx_opt),2);
        end
    end
end