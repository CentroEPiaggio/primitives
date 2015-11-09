classdef Elevate < primitive_library.PrimitiveFun
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
        function obj = Elevate(V,cost_coeff,cost_table,name,dimensions,default_extend)
            disp('dentro costruttore di Elevate')
            % Next line is a BITTERness! There are troubles inheriting the
            % PrimitiveFun constructor (which is executed before the Move constructor and apperas
            % not to read function arguments, i.e. always nargin=0). To
            % avoid this I used the function PrimitiveFun.Initialize to act
            % as a constructor.
            obj = obj.Initialize(V,cost_coeff,cost_table,name,dimensions,default_extend);
        end
        function [feasible,cost,q,x,time] = steering(obj,z_start,z_end)
            disp('E-le-va-tion!')
%             z_start
%             z_end
            
            % initialization
            feasible=false;
            cost=Inf;
            xi = z_start(1);
            vi = z_start(2);
            xf = z_end(1);
            vf = z_end(2);
            traj_pos_cart=NaN;
            traj_vel_cart=NaN;
            
            run_filepath = '../example/';
            prim_filepath = [run_filepath 'prim/'];
            
            Tend = 10; % TODO porcata. Il tempo va parametrizzato.
            Ts = 0.01;
                     
            yi = z_start(3);
            yf = z_end(3);
            
            % prepare data for simulation
            primitive_abbassa_params = struct('name','abbassa',    ...
                'yi',yi,            ...
                'yf',yf,            ...
                'ypi',0,            ...
                'ypf',0,            ...
                'Tend',Tend,        ...
                'Ts',Ts,            ...
                'yf_vec_len',1,     ...
                'filepath',prim_filepath ...
                );
            [time,traj_yp_cart]=gen_primitives_abbassa(primitive_abbassa_params);
            traj_y_cart = yi+cumtrapz(time,traj_yp_cart);
            % generate initial condition file for simulation
            %             xi = [0;0;0]; % HARDFIX
            xi = z_start(1);
            %             vi = [0;0;0]; % HARDFIX
            vi = z_start(2);
            traj_vel_cart = ones(size(time))*vi; % HARDFIX
            q0 = [xi(:);deg2rad(90);z_start(3)];
            q0 = [xi(:);deg2rad(90);traj_y_cart(1)];
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
            % disp(['zmp flag(end): ' num2str(rt_zmpflag(end))]) % display feasibility bit
            % disp('In steering_muovi:')
            % keyboard
%             keyboard
            if rt_zmpflag(end)==0
                feasible = 1;
                cost = rt_cost(end);
            else
                feasible = 0;
                cost = Inf;
            end
            % pack return data
            q = [traj_y_cart(1) traj_y_cart(end)];
            
            x = [traj_y_cart]';

        end
    end
end