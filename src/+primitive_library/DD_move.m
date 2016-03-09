classdef DD_move < primitive_library.PrimitiveFun
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
        function obj = DD_move(V,cost_coeff,cost_table,name,dimensions,default_extend,dimensions_imagespace,edge_color,ID,setup_parameters)
            cprintf('*[.6,0.1,1]*','dentro costruttore di DD_move\n');
            % Next line is a BITTERness! There are troubles inheriting the
            % PrimitiveFun constructor (which is executed before the DD_move constructor and apperas
            % not to read function arguments, i.e. always nargin=0). To
            % avoid this I used the function PrimitiveFun.Initialize to act
            % as a constructor.
            obj = obj.Initialize(V,cost_coeff,cost_table,name,dimensions,default_extend,dimensions_imagespace,edge_color,ID,setup_parameters);
        end
        function [feasible,cost,q,x,time] = steering(obj,z_start,z_end)
            cprintf('*[.6,0.1,1]*','Steering --> DD_move\n');
            disp(['From z_start: ' num2str(z_start(:)') ' to z_end: ' num2str(z_end(:)')])
            %             z_start
            %             z_end
            % STEERING_DD_MUOVI
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
            
            %% function [feasible,cost,q,traj_pos_cart,traj_vel_cart]=steering_dd_muovi(xi,xf,vi,vf)
            debug = false;
            % initialization
            feasible=false;
            cost=Inf;
            % state is like this: [x,y,theta,v]
            xi = z_start(1);
            yi = z_start(2);
            thi = z_start(3);
            vi = z_start(4);
            xf = z_end(1);
            yf = z_end(2);
            thf = z_end(3);
            vf = z_end(4);
            x=NaN;
            u=NaN;
            
            run_filepath = '../example/';
            prim_filepath = [run_filepath 'prim/'];
            
            Ts = 0.01;
            
            %yi = rand; %danilo briccone
            %yf = yi*2;
            % prepare data for muovi
            % define parameters for primitive muovi
            primitive_dd_muovi_params = struct('name','muovi',    ...
                'xi',xi,            ...
                'xf',xf,            ...
                'yi',yi,            ...
                'yf',yf,            ...
                'thi',thi,          ...
                'thf',thf,          ...
                'vi',vi,            ...
                'vf',vf,            ...
                'Ts',Ts,            ...
                'xf_vec_len',1, ...
                'vx0_vec_len',1,  ...
                'vxf_vec_len',1, ...
                'filepath',prim_filepath ...
                ); %                 'Tend',Tend,        ...
            %     [time,traj_x_cart]=gen_primitives_muovi(primitive_muovi_params);
            
            [time,x,u,q,retval,cost]=gen_primitives_dd_muovi_local(primitive_dd_muovi_params);
            
            if retval==1
                if debug
                    figure,plot(time,x,time(1)*ones(4,1),z_start,'ro',time(end)*ones(4,1),z_end,'ro'),grid on,legend('x','y','th','v')
                    keyboard
                end
                Tend = time(end);
                %                 traj_pos_cart = xi+cumtrapz(time,traj_vel_cart); % correctly returns the value of traj_pos_cart once traj_vel_cart has changed
                if any(isnan(time)) || any(any(isnan(u)))
                    feasible=0;
                    cost=Inf;
                    return
                end
                if nargout > 2 % if requested...
                    %                     traj_pos_cart = xi+cumtrapz(time,traj_vel_cart); % ...returns both the position trajectory...
                    %                     traj_vel_cart = traj_vel_cart;                % ...and the velocity trajectory
                    q = [xi x(end) vi u(end)];
                end
                
                % WARNING: we might not need to simulate this piece of
                % code, so it's fine to comment out the next lines.
                %                 traj_yp_cart = zeros(size(u));
                %
                %                 % generate initial condition file for simulation
                %                 q0 = [xi(:);deg2rad(90);2];
                %
                %                 qp0 = [vi(:);0;0];
                %                 qref0 = q0;
                %                 ic = struct('q0',q0, 'qp0',qp0, 'qref0',qref0);
                %                 gen_ic(ic);
                %                 % simulate
                %                 q_reference = [time(:)';
                %                     u(:)';
                %                     traj_yp_cart(:)'];
                %
                %                 save('../example/runna.mat','q_reference');
                %                 runstr = [run_filepath, 'modello -f rsim_tfdata.mat=' run_filepath 'runna.mat -p ' run_filepath 'params_steering.mat -v -tf ',num2str(Tend)];
                
                feasible = 1;
                
            else
                feasible = 0;
                q = NaN;
            end
        end
        % trimmed trajectories are with constant linear speed and null
        % angular speed
        function trimmed_trajectory = trim_trajectory(obj,z_start,time,x)
            verbose = 0;
            x_i = z_start(1);
            y_i = z_start(2);
            theta_i = z_start(3);
            v_i = z_start(4);
            w_i = 0;

            q_roomba = zeros(4,length(time));
            q_roomba(4,:) = kron(v_i,ones(size(time)));
            q_roomba(1:3,:) = kron([x_i; y_i; theta_i],ones(size(time))) + ([v_i*cos(theta_i); v_i*sin(theta_i); w_i])*time; % roomba is traveling at constant speed
            
            trimmed_trajectory = q_roomba;
            
            if verbose
                figure
                plot(time,q_roomba)
                grid on
                xlabel('time')
                legend('x','y','theta','v')
            end
        end
    end
end