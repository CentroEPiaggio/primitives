classdef ARM_move < primitive_library.PrimitiveFun
    properties
        A_g_0;    % TODO: this has to be parameterized
        shoulder_displacement; % TODO: this has to be parameterized
        L_arm;    % TODO: this has to be parameterized
        L_bar;    % TODO: this has to be parameterized
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
%         function obj = ARM_move(V,cost_coeff,cost_table,name,dimensions,default_extend,dimensions_imagespace,edge_color,ID,setup_parameters)
        function obj = ARM_move(V,name,dimensions,default_extend,dimensions_imagespace,edge_color,ID,setup_parameters)
            cprintf('*[.6,0.1,1]*','dentro costruttore di ARM_move\n');
            % Next line is a BITTERness! There are troubles inheriting the
            % PrimitiveFun constructor (which is executed before the DD_move constructor and apperas
            % not to read function arguments, i.e. always nargin=0). To
            % avoid this I used the function PrimitiveFun.Initialize to act
            % as a constructor.
%             obj = obj.Initialize(V,cost_coeff,cost_table,name,dimensions,default_extend,dimensions_imagespace,edge_color,ID,setup_parameters);
            obj = obj.Initialize(V,name,dimensions,default_extend,dimensions_imagespace,edge_color,ID,setup_parameters);
            
            % initialize goal position in inertial frame of reference
            obj.A_g_0 = eye(4);
%             x_target = 1; % HARDCODED
%             y_target = 0.4;
            x_target = setup_parameters.x_target;
            y_target = setup_parameters.y_target;
            obj.A_g_0(1:3,4) = [x_target;y_target;0.3];
            obj.shoulder_displacement = [0.1,-0.1,0.1]; % from arm_trajectory.m. TODO: parametrize?
            obj.L_arm = setup_parameters.L_arm;
            obj.L_bar = setup_parameters.L_bar;
        end
        function [feasible,cost,q,x,time] = steering(obj,z_start,z_end)
            cprintf('*[.6,0.1,1]*','Steering --> ARM_move\n');
            disp(['From z_start: ' num2str(z_start(:)') ' to z_end: ' num2str(z_end(:)')])
            %             z_start
            %             z_end
            % STEERING_ARM_move
            % this function takes as arguments the following:
            % z_start: the initial task value
            % z_end: the final task value
            
            debug = true;
            % initialization
            feasible=false;
            cost=Inf;
            % state is like this: [x,y,theta,v,tau]
            xi = z_start(1);
            yi = z_start(2);
            thi = z_start(3);
            vi = z_start(4);
            taui = z_start(5);
            xf = z_end(1);
            yf = z_end(2);
            %             thf = z_end(3);
            %             vf = z_end(4);
            %             tauf = z_end(5);
            tauf = z_end(3);
            x=NaN;
            u=NaN;
            
            Ts = 0.1;
            %% Humanoid trajectories
%             keyboard
%             arm_speed = 0.5; % mean speed
%             distance = (norm(z_end(obj.dimensions>0) - z_start(obj.dimensions>0)))*obj.L_arm;
            time = 0:Ts:1;
            q = [z_start(:)' z_end(:)'];
            if tauf == taui
                feasible = false;
                return
            end
            x = linspace(taui,tauf,length(time));
            cost = time(end);
            if cost == 0
                feasible = false;
                return
            end
            feasible = true;
            
            return
            %%
            if isnan(z_start(5)) % TODO: FIXME!!!
                q0_arm = [0;0;0;0];
            else
                q0_arm = [0;0;0;0];
            end
            %yi = rand; %danilo briccone
            %yf = yi*2;
            % prepare data for muovi
            % define parameters for primitive muovi
            primitive_arm_muovi_params = struct('name','muovi',    ...
                'xi',xi,            ...
                'xf',xf,            ...
                'yi',yi,            ...
                'yf',yf,            ...
                'thi',thi,          ... % 'thf',thf,          ...
                'vi',vi,            ... % 'vf',vf,            ...
                'taui',taui,            ...
                'tauf',tauf,            ...
                'A_g_0',obj.A_g_0,      ...
                'q0_arm',q0_arm,    ...
                'Ts',Ts            ...
                ); %                 'Tend',Tend,        ...
            
%             if debug
%                 keyboard
%             end
            [time,x,u,q,retval,cost]=gen_primitives_arm_move_local(primitive_arm_muovi_params);
            
            %keyboard
            feasible = ~retval; % sick convention, retval is 1 on failure.
            if feasible
                if isnan(taui)
                    taui = 0; % HARDCODED INITIALIZATION
                end
                tauf = x(end);
                x = linspace(taui,tauf,length(time));
                cost = time(end);
                keyboard
            end
            
            return
            
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
        % trimmed trajectories are constant arm joint angles
        function trimmed_trajectory = trim_trajectory(obj,z_start,time,x)
            verbose = 0;
%             keyboard
            if isnan(z_start(obj.dimensions>0)) % if this primitive has not been activated yet, just don't use it
                trimmed_trajectory = nan(size(time));
                return
            end
            trimmed_trajectory = kron(z_start(obj.dimensions>0),ones(size(time)));
            
            return
            % arm parameters
%             L_arm = 0.31; % maximum radius of reachability region of the arm w.r.t. base frame, i.e. sum of length of the links
            
            % FIXME: q_arm_trimmered has to come from somewhere else
            q_arm_trimmered = [0;0;0;0;0]; % this primitive, when trimmered, keeps the joints still.
            
            %%
            inertial_for_coordinates = blkdiag(eye(3),0);
            q_roomba = x(1:3,:);
            CoM_coordinates = zeros(3,length(time));
            A_b_0 = zeros(4,4,length(time));
            [~,A_s_b] = DK_s_b(obj.shoulder_displacement);
            A_s_0 = zeros(4,4,length(time));
            p_g_0 = [obj.A_g_0(:,4)]; % goal position in inertial reference frame
            tau = zeros(1,length(time));
            for ii=1:length(time)
                CoM_coordinates(:,ii) = q_roomba(1:3,ii);
                [~,A_b_0(:,:,ii)] = DK_b_0(CoM_coordinates(:,ii));
                A_s_0(:,:,ii) = A_b_0(:,:,ii)*A_s_b;
                goal_position_s = A_s_0(:,:,ii)\p_g_0; % goal position in shoulder reference frame
                
                [~,~,~,A_e_s] = DK_arm(q_arm_trimmered);
                
                A_e_0 = A_s_0(:,:,ii)*A_e_s;
                
                p_e_0 = A_e_0(:,4);
                
                dist_euclid_goal_ee = norm(p_g_0 - p_e_0); % euclidean distance
                distance = 1-dist_euclid_goal_ee/obj.L_arm;
                tau(ii) = max(0,min(1,distance)); % tau belongs to [0,1]
            end
            
            trimmed_trajectory = tau;
            
            if verbose
                figure
                plot(time,trimmed_trajectory)
                grid on
                xlabel('time [s]')
                legend('tau')
            end
        end
    end
end