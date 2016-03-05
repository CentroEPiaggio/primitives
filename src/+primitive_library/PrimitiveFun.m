classdef PrimitiveFun
    properties
        name; % string
        chi; % Imagespace
%         Params q;
        q; % primitive parameters
%         Mapping f;
        cost_coeff;  % replace with an abstract function or sth else
        cost_table;
        dimensions; % tracks what dimensions are used by a primitive with its control % dimensions that are directly changed when this primitive is active (others are trimmed by other primitives)
        dimensions_imagespace; % i.e. where the sampling is allowed, used in CheckAvailablePrimitives
        initial_extend; % initial value used when expanding the primitive (i.e. the hyperplane where the projection happens)
        edge_color; % color of the edges when plotting the tree
        ID;
    end
    methods
        % constructor
%         function obj = PrimitiveFun(V,cost_coeff,cost_table,name,dimensions,initial_extend) % TODO: PrimitiveFun(chi,q,f)
            function obj = Initialize(obj,V,cost_coeff,cost_table,name,dimensions,initial_extend,dimensions_imagespace,edge_color,ID) % TODO: PrimitiveFun(chi,q,f)
%             disp('Dentro costruttore di PrimitiveFun');
            if nargin >= 8 % obj takes one argument, the others are our parameters
                obj.chi = primitive_library.Imagespace(V);
                obj.cost_coeff = cost_coeff;
                obj.cost_table = cost_table;
                obj.name = name;
                obj.dimensions = dimensions;
                obj.dimensions_imagespace = dimensions_imagespace;
                obj.initial_extend = initial_extend;
                obj.edge_color = 'blue';
            elseif nargin < 1
                obj.chi = Imagespace([-1 -1; -1 1; 1 -1; 1 1]*0.3);
            else
                error('Error in initialization of primitives!');
            end
            if nargin == 10
                obj.edge_color = edge_color;
                obj.ID = ID;
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
        % check if a point can be extended
        function extendable = check_extendable(obj,z_test) % z_test must be already with its own NaNs
            z_check = z_test;
            z_check(~isnan(obj.initial_extend)) = obj.initial_extend(~isnan(obj.initial_extend));
            z_check = z_check(obj.dimensions_imagespace>0);
            extendable = obj.chi.P.contains(z_check);
        end
        % extend a point in the current primitive image space by replacing
        % (if there are) NaN values with initial_extend values
        function extended_state = extend(obj,z_test) % z_test must be already with its own NaNs
%             keyboard
            nans = isnan(z_test);
            nans_wholedimensions = [~nans(:); zeros(length(obj.dimensions)-length(nans),1)];
            if isequal(nans_wholedimensions,obj.dimensions(:)) %|| ~any(nans)
                extended_state = z_test; % do nothing
                return
            end
            % here we need to initialize some nan values to the
            % initial_extend value
            extended_state = z_test;
            
            if obj.name == 'ARM_move' % HARDCODED
                extended_state(5) = 0;
            else
                if isnan(z_test(isnan(obj.initial_extend)>0))
                    extended_state((~isnan(obj.initial_extend))>0) = obj.initial_extend((~isnan(obj.initial_extend))>0);
                end
            end
            
            %             keyboard
        end
    end
    methods (Abstract)
        % this function is the steering function for the primitive:
        % it has to be implemented in every
        steering(obj,z_start,z_end)
        % this function expresses the behavior of the primitive when it is
        % trimmed (i.e., how the space state evolves under null input
        trim_trajectory(obj,z_start,time,x)
    end
end