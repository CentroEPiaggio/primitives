classdef Move < primitive_library.PrimitiveFun
    properties
        %         Control u; % generic control law
        %         Trig xi; % trigger conditions
        %         Duration t; % duration of execution of the primitive
        %         Params q; % task-specific parameters
        %         Imagespace chi; % image space of the primitive: see Planning_notes.tex for insight.
        %                         % These should be treated with a suitable (and easy to use) library
        %                         % as YALMIP.
        %         PrimitiveFun f; % The function that maps the params q to the imagespace chi.
    end
    methods
        function obj = Move(V,cost_coeff,cost_table,name,dimensions,default_extend,edge_color,ID)
            disp('dentro costruttore di Move')
            % Next line is a BITTERness! There are troubles inheriting the
            % PrimitiveFun constructor (which is executed before the Move constructor and apperas
            % not to read function arguments, i.e. always nargin=0). To
            % avoid this I used the function PrimitiveFun.Initialize to act
            % as a constructor.
            obj = obj.Initialize(V,cost_coeff,cost_table,name,dimensions,default_extend,edge_color,ID);
        end
        function [feasible,cost,q,x,time] = steering(obj,z_start,z_end)
            disp('Move like Jagger!')
            disp(['From z_start: ' num2str(z_start(:)') ' to z_end: ' num2str(z_end(:)')])
            %             z_start
            %             z_end
            % STEERING_MUOVI
            % this function takes as arguments the following:
            % enable_muovi: true or false
            % enable_alza:  true or false
            % xi: horizontal initial position
            % xf: horizontal final position
            % yi: vertical initial position
            % yf: vertical final position
            % the function returns the following:
            % feasible: true or false
            % cost: the cost of performing such action
            % data: the data to be added to the plan tree?
            
            %% function [feasible,cost,q,traj_pos_cart,traj_vel_cart]=steering_muovi(xi,xf,vi,vf)
            
            % initialization
            feasible=false;
            cost=Inf;
            % q=[xi,xf,vi,vf];
            xi = z_start(1);
            vi = z_start(2);
            xf = z_end(1);
            vf = z_end(2);
            traj_pos_cart=NaN;
            traj_vel_cart=NaN;
            
            run_filepath = '../example/';
            prim_filepath = [run_filepath 'prim/'];
            
%             Tend = 10; % TODO porcata. Il tempo va parametrizzato. % DONE
            Ts = 0.001;
            
            yi = rand;
            yf = yi*2;
            % prepare data for muovi
            % define parameters for primitive muovi
            primitive_muovi_params = struct('name','muovi',    ...
                'xi',xi,            ...
                'xf',xf,            ...
                'vi',vi, ...
                'vf',vf, ...
                'Ts',Ts,            ...
                'xf_vec_len',1, ...
                'vx0_vec_len',1,  ...
                'vxf_vec_len',1, ...
                'filepath',prim_filepath ...
                ); %                 'Tend',Tend,        ...
            %     [time,traj_x_cart]=gen_primitives_muovi(primitive_muovi_params);
            
            [time,traj_pos_cart,traj_vel_cart,q,retval]=gen_primitives_muovi_local(primitive_muovi_params);
            
            if retval==1
                Tend = time(end);
%                 traj_pos_cart = xi+cumtrapz(time,traj_vel_cart); % correctly returns the value of traj_pos_cart once traj_vel_cart has changed
                if any(isnan(time)) || any(isnan(traj_vel_cart))
                    feasible=0;
                    cost=Inf;
                    return
                end
                if nargout > 2 % if requested...
%                     traj_pos_cart = xi+cumtrapz(time,traj_vel_cart); % ...returns both the position trajectory...
%                     traj_vel_cart = traj_vel_cart;                % ...and the velocity trajectory
                    q = [xi traj_pos_cart(end) vi traj_vel_cart(end)];
                end
                
                traj_yp_cart = zeros(size(traj_vel_cart));
                
                % generate initial condition file for simulation
                q0 = [xi(:);deg2rad(90);2];
                
                qp0 = [vi(:);0;0];
                qref0 = q0;
                ic = struct('q0',q0, 'qp0',qp0, 'qref0',qref0);
                gen_ic(ic);
                % simulate
                q_reference = [time(:)';
                    traj_vel_cart(:)';
                    traj_yp_cart(:)'];
                save('../example/runna.mat','q_reference');
                runstr = [run_filepath, 'modello -f rsim_tfdata.mat=' run_filepath 'runna.mat -p ' run_filepath 'params_steering.mat -v -tf ',num2str(Tend)];
                [status, result] = system(runstr);
                if status ~= 0, error(result); end
                % check if feasible
                load('../src/modello.mat');
% keyboard                 % stop here to see simulated trajectories
                if rt_zmpflag(end)==0
                    feasible = 1;
%                     cost = rt_cost(end);
                    cost = Tend; % for time-optimal problem
                    cprintf('*[0,1,0]*','! OK ZMP check !\n');
                else
                    feasible = 0;
                    cost = Inf;
%                     disp('Failed ZMP check');
                    cprintf('*[1,0,0]*','! Failed ZMP check !\n');
                    keyboard % stop here to check simulated trajectories for wrong reference trajectories with: figure,plot(rt_t,rt_qp_ref)
                end
                % pack return data
                x = [traj_pos_cart;traj_vel_cart];
            else
                feasible = 0;
                cost = Inf;
                x = NaN;
                time = NaN;
                q = NaN;
            end
        end
    end
end