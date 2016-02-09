function [time,pos,speed,acc,jerk,retval,cost] = dd_optimal_trajectory(x0,xf,Ts,state_bounds,control_bounds)
debug = 0;
verbose = 0;

time = [];
pos = [];
speed = [];
acc = [];
jerk = [];

% ensure vectors are column
x0 = x0(:);
xf = xf(:);
control_bounds = control_bounds(:);
if length(control_bounds) == 2
    lower_control_bound = control_bounds(1);
    upper_control_bound = control_bounds(2);
else
    lower_control_bound = -abs(control_bounds(1));
    upper_control_bound = abs(control_bounds(1));
end

if size(state_bounds,2) == 2
    lower_state_bound = state_bounds(:,1);
    upper_state_bound = state_bounds(:,2);
elseif size(state_bounds,2) == 1
    lower_state_bound = -abs(state_bounds(:,1));
    upper_state_bound = abs(state_bounds(:,1));
else
    disp('Error in state constraint specification');
    retval = -2;
    return
end
% default time step

%%
found = 0;

%% analytic solution

% from D.J. Balkcom, M.T. Mason - Time Optimal Trajectories for Bounded
% Velocity Differential Drive Vehicles.
% The goal is considered as the origin of the reference frame to compute
% the optimal control w.r.t. the start configuration

new_x0 = x0+xf;
new_xf = xf*0;

x = new_x0(1);
y = new_x0(2);
th = new_x0(3);

if pi < th && th < 2*pi
    new_x0(1:3)=[new_x0(1);-new_x0(2);-new_x0(3)]; %T3
    x = new_x0(1);
    y = new_x0(2);
    th = new_x0(3);
end

r = norm(x,y);
z = atan2(y,x);

if ( (th+pi)/2 < z && z < pi ) || ( (th-pi)/2 < z && z < 0)
    rot_th = [cos(th) -sin(th); sin(th) cos(th)];
    new_x0(1:2) = rot_th*[new_x0(1); -new_x0(2)]; %T2
    x = new_x0(1);
    y = new_x0(2);
    th = new_x0(3); 
end

if y < 0
   new_x0(1:3)=[-new_x0(1);-new_x0(2);new_x0(3)]; %T1
   x = new_x0(1);
   y = new_x0(2);
   th = new_x0(3); 
end

% compute cost
cost = 0;

if z <= th
    % turn right, backward, turn right - TST (turn, straight, turn)
    cost = r+min(abs(z)+abs(z-th),2*pi+abs(z)-abs(z+th));

else if y <= 1-cos(th)
        % backward, turn right, forward - STS
        if y==0
            cost = abs(x)+th/2;
        else
            cost = y*(1+cos(th))/sin(th)-x+th/2;
        end
        
    else if r >= tan(z/2)
            % turn left, backward, turn right - TST
            cost = r+min(abs(z)+abs(z-th),2*pi+abs(z)-abs(z+th));
            
        else
            % turn left, backward, turn right, forward - TSTS
            cost = acos(1-y)-th/2-x+sqrt(y*(2-y));
            
        end 
    end
end

time = 0;
pos = 0;
speed = 0;
acc = 0;
jerk = 0;

%%
retval = 1; % success