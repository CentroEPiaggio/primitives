function [time,pos,speed,acc,jerk,retval,cost] = dd_optimal_trajectory(x0,xf,Ts,state_bounds,control_bounds)
debug = 1;
verbose = 1;

time = [];
pos = [];
speed = [];
acc = [];
jerk = [];

% ensure vectors are column
x0 = x0(:);
xf = xf(:);

% default time step

%% analytic solution

% from D.J. Balkcom, M.T. Mason - Time Optimal Trajectories for Bounded
% Velocity Differential Drive Vehicles. - Cap.11 procedures.
% The goal is considered as the origin of the reference frame to compute
% the optimal control w.r.t. the start configuration

new_x0 = x0+xf;
new_xf = xf*0;

x = new_x0(1);
y = new_x0(2);
th = new_x0(3);

for i=1:1000 % reproduce the recursive procedure;

    if pi < th && th < 2*pi
        new_x0(1:3)=[new_x0(1);-new_x0(2);-new_x0(3)]; %T3
        x = new_x0(1);
        y = new_x0(2);
        th = new_x0(3);
        if verbose cprintf('*[1,0.7,0]*',' - applying T3\n'); end
        continue;
    end

    r = norm(x,y);
    z = atan2(y,x);

    if ( (th+pi)/2 < z && z < pi ) || ( (th-pi)/2 < z && z < 0)
        rot_th = [cos(th) -sin(th); sin(th) cos(th)];
        new_x0(1:2) = rot_th*[new_x0(1); -new_x0(2)]; %T2
        x = new_x0(1);
        y = new_x0(2);
        th = new_x0(3);
        if verbose cprintf('*[1,0.7,0]*',' - applying T2\n'); end
        continue;
    end

    if y < 0
       new_x0(1:3)=[-new_x0(1);-new_x0(2);new_x0(3)]; %T1
       x = new_x0(1);
       y = new_x0(2);
       th = new_x0(3);
       if verbose cprintf('*[1,0.7,0]*',' - applying T1\n'); end
       continue;
    end

end
% compute type of trajectory and cost

if z <= th
    % turn right, backward, turn right - TST (turn, straight, turn)
    if verbose cprintf('*[1,0,0]*',' - TST (class D)\n'); end
    cost = r+min(abs(z)+abs(z-th),2*pi+abs(z)-abs(z+th));

else if y <= 1-cos(th)
        % backward, turn right, forward - STS
        if verbose cprintf('*[0.2,1,0.2]*',' - STS (class G)\n'); end
        if y==0
            cost = abs(x)+th/2;
        else
            cost = y*(1+cos(th))/sin(th)-x+th/2;
        end
        
    else if r >= tan(z/2)
            % turn left, backward, turn right - TST
            if verbose cprintf('*[1,0,1]*',' - TST (class F)\n'); end
            cost = r+min(abs(z)+abs(z-th),2*pi+abs(z)-abs(z+th));
            
        else
            % turn left, backward, turn right, forward - TSTS
            if verbose cprintf('*[0,0,1]*',' - TSTS (class H)\n'); end
            cost = acos(1-y)-th/2-x+sqrt(y*(2-y));
            
        end 
    end
end

error = Inf;
threshold = 0.1;
b=0.3;
x_t=[];
x_t = [x_t x0(1)];
y_t=[];
y_t = [y_t x0(2)];
th_t=[];
th_t = [th_t x0(3)];
index = 1;

while error>threshold
    
    x_w1 = x_t(index) - b*sin(th_t(index));
    y_w1 = x_t(index) + b*cos(th_t(index));
    x_w2 = x_t(index) + b*sin(th_t(index));
    y_w2 = x_t(index) - b*cos(th_t(index));

    w1 = - sign( -1/(2*b) * eta(x_w2,y_w2) );
    w2 = - sign( 1/(2*b) * eta(x_w1,y_w1) );
    
    v = (w1+w2)/2;
    w = (w2-w1)/(2*b);
    
    % integration
    % x(t) = x0 + int[0,T]{ v cos(th) }
    % y(t) = y0 + int[0,T]{ v sin(th) }
    % th(t) = th0 + int[0,T]{ w }
    
    x_t = [x_t x_t(index) + Ts*v*cos(th_t(index))];
    y_t = [y_t y_t(index) + Ts*v*sin(th_t(index))];
    th_t = [th_t th_t(index) + Ts*w];

    error = norm(xf-[x_t(end) y_t(end) th_t(end)].')
    
    index = index +1;
end

time = 0;
pos = 0;
speed = 0;
acc = 0;
jerk = 0;

%%
retval = 1; % success
end

function out=eta(x,y)
    c1 = 0.4;
    c2 = sqrt (1 - c1^2);
    c3 = 0.2;
    out = c1*y -c2*x + c3;
end