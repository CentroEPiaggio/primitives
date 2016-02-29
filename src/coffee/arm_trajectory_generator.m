% ARM_TRAJECTORY_GENERATOR generates trajectory for joints of OWA robot
%
% PARAMETERS TO TUNE:
% - max_joint_speed: rad/s
% - max_joint_acceleration: rad/s^2
% - max_joint_jerk: rad/s^3
% RETURNS:
% - flag: 1 on error, 0 on success
% NOTATION: A_1_0: transformation matrix from frame 0 to frame 1, e.g.
% A_b^0 is base frame expressed in inertial coordinates

function [flag,time,tau,traj_q,traj_qp]=arm_trajectory_generator(Ts,q_roomba_0,A_g_0,distance_from_goal,q0_arm)
debug = 1;
verbose = 1;
% Initialization
flag = 1; % initialize with failed state
time = [];
traj_q = [];
traj_qp = [];
tau = [];
% Tuning parameters for the
max_joint_speed = 2;
max_joint_acceleration = 2;
max_joint_jerk = 2;
%% Compute final position w.r.t. goal's position
% R_0: inertial frame of reference
inertial_for_coordinates = blkdiag(eye(3),0);
% R_b: robot floating base of reference
CoM_coordinates_0 = q_roomba_0(1:3);
[~,A_b_0] = DK_b_0(CoM_coordinates_0);
% R_s: robot's arm base frame of reference w.r.t. R_b
shoulder_displacement = [0.1,-0.1,0.1];
[~,A_s_b] = DK_s_b(shoulder_displacement);
A_s_0 = A_b_0*A_s_b;
% % R_g_s: goal position in shoulder frame of reference
% [~,A_g_s] = DK_g_0(goal_position_s);
% p_g_s = [goal_position_s(1:3)];
% % find an arm position at a specified distance from the goal
% A_g_0 = A_s_0*A_g_s;
% R_g_s: goal position in shoulder frame of reference
A_g_s = A_s_0\A_g_0;
goal_position_s = A_g_s(1:3,4);
p_g_s = [goal_position_s(1:3)];

L_arm = 0.31; % maximum radius of reachability region of the arm w.r.t. base frame, i.e. sum of length of the links
if norm(goal_position_s)>L_arm
    disp('Over the hills and far away');
    if debug
        keyboard
    end
    return
end

goal_position_0 = A_g_0(1:3,4);


p_s_0 = A_s_0(1:3,4);
p_g_0 = A_g_0(1:3,4);

% versore = p_g_s/norm(p_g_s);
% keyboard
approach_angle = []; % constrain this to +- 45 degrees. It is done in IK_arm_simple

% End-effector position in shoulder frame of reference
lam = distance_from_goal/(norm(p_g_s(1:3)));
lam = min(1,max(0,lam));
% p_e_desired_s = ([0;0;p_g_s(3)])*(lam) + p_g_s*(1-lam);
% p_e_desired_s(3) = p_g_s(3); % constrain the height of the end effector to be the same of the goal
rest_position_s = [0.15;0;0.15];
p_e_desired_s = rest_position_s*(lam) + p_g_s*(1-lam);
p_e_desired_s = p_g_s;

% End-effector position in inertial frame of reference
p_e_desired_0 = A_s_0*[p_e_desired_s;1];

% find joint coordinates with inverse kinematics
test_point = p_e_desired_s(1:3);
[theta_arm,flag] = IK_arm_simple(test_point(1),test_point(2),test_point(3),approach_angle); % DEGREES!!!

%
if flag == 1
    if verbose
        disp('------- Error in IK! -------')
    end
    if debug
        keyboard
    end
    return
else
    q_arm = deg2rad(theta_arm);
    if verbose
        disp('------- IK feasible! -------')
    end
    [~,~,~,A_e_s] = DK_arm(q_arm);
    % R_e: end effector frame of reference w.r.t. inertial frame of
    % reference
    A_e_0 = A_s_0*A_e_s;
    p_e_s = A_e_s(1:3,4);
    p_e_0 = A_e_0(1:3,4);
    distance_obtained = norm(p_g_0-p_e_0);
    estimated_distance = norm(p_g_0-p_e_desired_0(1:3));
    disp(['Desired distance: ' num2str(distance_from_goal) '. Obtained one is: ' num2str(distance_obtained) '. Estimated one: ' num2str(estimated_distance)]);
    tau = 1-distance_obtained/L_arm;
    if tau>1 || tau<0 || distance_from_goal~=distance_obtained
        disp('ARM_TRAJECTORY_GENERATION: Sampled a strange point!');
        if tau>1 || tau<0
            flag = 1;
        end
        if debug
            figure
            plot_rf(inertial_for_coordinates,'R_0')
            hold on
            plot_rf(A_b_0,'R_b')
            plot_rf(A_s_0,'R_s')
            plot_rf(A_g_0,'R_g')
            plot_rf(A_e_0,'R_e')
            plot_arm(q_arm,A_s_0)
            axis equal
            title('Expected final configuration (at 0 vehicle speed)')
            xlabel('x(m)'),ylabel('y(m)'),zlabel('z(m)');
            plot3(p_e_desired_0(1),p_e_desired_0(2),p_e_desired_0(3),'rx','linewidth',2)
            p_rest_position_0 = A_s_0*[rest_position_s;1];
            plot3(p_rest_position_0(1),p_rest_position_0(2),p_rest_position_0(3),'kx','linewidth',2)
            keyboard
        end
%         return
    end
end

p_g_e = A_g_0(1:3,4) - A_e_0(1:3,4)
%% generate trajectory
% q0 = [0;0;0;0];
% q0 = q0_arm;
% qf = q_arm;
% 
% figure(10); clf;
% jj=1;
% for ii=1:length(q_arm)
%     xi{ii} = q0(ii);
%     xf{ii} = qf(ii);
%     if xi{ii}==xf{ii}
%         time{ii} = 0;
%         traj_q{ii} = xi{ii};
%         traj_qp{ii} = 0;
%         retval{ii}=1;
%         continue;
%     end
%     xpi = 0;
%     xpf = 0;
%     
%     vmax = max_joint_speed;
%     amax = max_joint_acceleration;
%     jerkmax = max_joint_jerk;
%     
%     x_0 = [xi{ii};xpi;0];
%     x_f = [xf{ii};xpf;0];
%     state_bounds = [-Inf Inf;
%         -vmax vmax;
%         -amax amax];
%     control_bounds = [-jerkmax;jerkmax];
%     
%     [time{ii},traj_q{ii},traj_qp{ii},~,~,retval{ii}] = min_jerk_trajectory_analytic(x_0,x_f,Ts,state_bounds,control_bounds);
%     
%     if retval{ii}
%         time{ii} = time{ii}(:)';
%         traj_q{ii}  = traj_q{ii} (:)';
%         traj_qp{ii}  = traj_qp{ii} (:)';
%     else
%         disp(['------- Error on trajectory generation of joint ' num2str(ii) ' -------'])
%         flag = 1;
%         if debug
%             keyboard
%         end
% %         return
%     end
% end

% 
% flag = 0; % at this point all trajectories have been found
% 
% T_max = max([time{:}]); % duration of the slowest joint trajectory
% 
% for kk=1:length(q_arm)
%     % extend trajectories to cope with the slowest one
%     time_extension = time{kk}(end)+Ts:Ts:T_max;
%     time{kk} = [time{kk} time_extension]; % extend time
%     traj_q{kk} = [traj_q{kk}, traj_q{kk}(end)*ones(size(time_extension))]; % extend time
%     traj_qp{kk} = [traj_qp{kk}, zeros(size(time_extension))]; % extend time
% end
% % compute trajectories
% q_roomba_f = q_roomba_0;
% q_roomba_f(1:2) = q_roomba_0(1:2) + q_roomba_0(4)*T_max*[cos(q_roomba_0(3));sin(q_roomba_0(3))]; % roomba is traveling at constant speed
% t = time{1}; % all time vectors are equal after being extended to wait for the slowest one
% q_roomba = kron(q_roomba_0(1:3),ones(size(t))) + (q_roomba_0(4)*[cos(q_roomba_0(3));sin(q_roomba_0(3));q_roomba_0(5)])*t; % roomba is traveling at constant speed
% q_arm_traj = [traj_q{:}];
% q_arm_traj = reshape(q_arm_traj,[length(t),length(q_arm)])';
% 
% qp_arm_traj = [traj_qp{:}];
% qp_arm_traj = reshape(qp_arm_traj,[length(t),length(q_arm)])';
% 
% %% pack data for return
% traj_q = q_arm_traj;
% traj_qp = qp_arm_traj;
% time = t;
%% plot
if verbose
    figure
    plot_rf(inertial_for_coordinates,'R_0')
    hold on
    plot_rf(A_b_0,'R_b')
    plot_rf(A_s_0,'R_s')
    plot_rf(A_g_0,'R_g')
    if flag==0
        plot_rf(A_e_0,'R_e')
        plot_arm(q_arm,A_s_0)
    end
    axis equal
    title('Expected final configuration (at 0 vehicle speed)')
    xlabel('x(m)'),ylabel('y(m)'),zlabel('z(m)');
end

Ts = 0.01;
time = 0:Ts:1;
tau = linspace(0,1,length(time));
%%
if debug
    disp(' Sampled distance ' )
    distance_from_goal
    disp(' Calculated distance ' )
    norm(p_g_e)
end

end