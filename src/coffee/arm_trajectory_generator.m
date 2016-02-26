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

function [flag,time,traj_q,traj_qp,A_g_0]=arm_trajectory_generator(Ts,q_roomba_0,goal_position_s,distance_from_goal)
debug = 1;
verbose = 1;
% Initialization
flag = 1; % initialize with failed state
% Tuning parameters for the
max_joint_speed = 1;
max_joint_acceleration = 1;
max_joint_jerk = 1;
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
% R_g_s: goal position in shoulder frame of reference
[~,A_g_s] = DK_g_0(goal_position_s);
p_g_s = [goal_position_s(1:3)];
% find an arm position at a specified distance from the goal
A_g_0 = A_s_0*A_g_s;

goal_position_0 = A_g_0(1:3,4);

approach_angle = 0;
p_s_0 = A_s_0(1:3,4);
p_g_0 = A_g_0(1:3,4);

% End-effector position in shoulder frame of reference
lam = distance_from_goal/(norm(p_g_s(1:2)));
p_e_desired_s = ([0;0;p_g_s(3)])*(lam) + p_g_s*(1-lam);

p_e_desired_s(3) = p_g_s(3); % constrain the height of the end effector to be the same of the goal
% End-effector position in inertial frame of reference
p_e_desired_0 = A_s_0*[p_e_desired_s;1];

% find joint coordinates with inverse kinematics
test_point = p_e_desired_s(1:3);
[theta_arm,flag] = IK_arm(test_point(1),test_point(2),test_point(3),approach_angle); % DEGREES!!!

%
if flag == 1
    if verbose
        disp('------- Error in IK! -------')
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
end

p_g_e = A_g_0(1:3,4) - A_e_0(1:3,4)
%% generate trajectory
q0 = [0;0;0;0];
qf = q_arm;

figure(10); clf;
jj=1;
for ii=1:length(q_arm)
    xi{ii} = q0(ii);
    xf{ii} = qf(ii);
    xpi = 0;
    xpf = 0;
        
    vmax = max_joint_speed;
    amax = max_joint_acceleration;
    jerkmax = max_joint_jerk;
    
    x_0 = [xi{ii};xpi;0];
    x_f = [xf{ii};xpf;0];
    state_bounds = [-Inf Inf;
        -vmax vmax;
        -amax amax];
    control_bounds = [-jerkmax;jerkmax];
    
    [time{ii},traj_q{ii},traj_qp{ii},~,~,retval{ii}] = min_jerk_trajectory_analytic(x_0,x_f,Ts,state_bounds,control_bounds);
    
    if retval{ii}
        time{ii} = time{ii}(:)';
        traj_q{ii}  = traj_q{ii} (:)';
        traj_qp{ii}  = traj_qp{ii} (:)';
    else
        disp(['------- Error on trajectory generation of joint ' num2str(ii) ' -------'])
        flag = 1;
        return
    end
end

flag = 0; % at this point all trajectories have been found

T_max = max([time{:}]); % duration of the slowest joint trajectory

for kk=1:length(q_arm)
    % extend trajectories to cope with the slowest one
    time_extension = time{kk}(end)+Ts:Ts:T_max;
    time{kk} = [time{kk} time_extension]; % extend time
    traj_q{kk} = [traj_q{kk}, traj_q{kk}(end)*ones(size(time_extension))]; % extend time
    traj_qp{kk} = [traj_qp{kk}, zeros(size(time_extension))]; % extend time
end
% compute trajectories
q_roomba_f = q_roomba_0;
q_roomba_f(1:2) = q_roomba_0(1:2) + q_roomba_0(4)*T_max*[cos(q_roomba_0(3));sin(q_roomba_0(3))]; % roomba is traveling at constant speed
t = time{1}; % all time vectors are equal after being extended to wait for the slowest one
q_roomba = kron(q_roomba_0(1:3),ones(size(t))) + (q_roomba_0(4)*[cos(q_roomba_0(3));sin(q_roomba_0(3));q_roomba_0(5)])*t; % roomba is traveling at constant speed
q_arm_traj = [traj_q{:}];
q_arm_traj = reshape(q_arm_traj,[length(t),length(q_arm)])';

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

%%
if debug
    disp(' Sampled distance ' )
    distance_from_goal
    disp(' Calculated distance ' )
    norm(p_g_e)
end

end