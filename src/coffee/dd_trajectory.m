function [time,pos,speed,acc,jerk,retval,cost] = dd_trajectory(x0,xf,Ts,state_bounds,control_bounds)
debug = 1;
verbose = 1;

if verbose
    tic
end

time = [];
pos = [];
speed = [];
acc = [];
jerk = [];

% ensure vectors are column
x0 = x0(:);
xf = xf(:);

cost=Inf;

x_i=x0(1);
y_i=x0(2);
th_i=x0(3);
v_i=x0(4);

x_f=xf(1);
y_f=xf(2);
th_f=xf(3);
v_f=xf(4);

if v_f==0
    v_f=1e-5; %numerical stuff
end

% Using Bezier curves

control_factor = 0.7;

% the trick of the century: I add a control point after the start, and a 
% control point before the end, to enforce bezier starting and ending
% orientation

P = [x_i x_i+control_factor*cos(th_i) x_f-control_factor*cos(th_f) x_f; 
     y_i y_i+control_factor*sin(th_i) y_f-control_factor*sin(th_f) y_f];

pt1 = P(:,1);
pt2 = P(:,2);
pt3 = P(:,3);
pt4 = P(:,4);

t = linspace(0,1,101);
c_u = kron((1-t).^3,pt1) + kron(3*(1-t).^2.*t,pt2) + kron(3*(1-t).*t.^2,pt3) + kron(t.^3,pt4);

if debug
    figure
    subplot(1,3,1)
    hold on
    set (gcf, 'Units', 'normalized', 'Position', [0,0,1,1]);
    plot(P(1,1),P(2,1),'*r'), title('Trajectory with waypoints')
    plot(P(1,end),P(2,end),'+r')
    plot(P(1,2:end-1),P(2,2:end-1),'*b')
    plot(c_u(1,:),c_u(2,:),'Linewidth',2)
    axis equal
end

% trajectory following

threshold = 1e-2; %this is to check the error between the current and desired position
f_threshold = 1e-1; %this is to start execute the last maneuvers before ending the trajectory (e.g. converge to v_f)

x_t=[];
x_t = [x_t x_i];
y_t=[];
y_t = [y_t y_i];
th_t=[];
th_t = [th_t th_i];

K1=1;
K2=1;
b=0.3;

index = 1;
wp=[];
for i=1:numel(t)
    if mod(i,10)==0
        wp = [wp c_u(:,i+1)];
    end
end

w_max=1;
w_sat = saturation([-w_max w_max]);
v_max=0.1;
v_sat = saturation([-v_max v_max]);
v=v_i;
w=0;

pos_ok = false;
vel_ok = true;

retval=1;

for i=1:size(wp,2)
        error = norm(wp(:,i)-[x_t(end) y_t(end)].');
        pos_ok = error<threshold;
        if i==size(wp,2)
            vel_ok=false;
        end

        while ~pos_ok || ~vel_ok

            speed = [speed, [v; w]];

            % integration
            % x(t) = x0 + int[0,T]{ v cos(th) }
            % y(t) = y0 + int[0,T]{ v sin(th) }
            % th(t) = th0 + int[0,T]{ w }

            x_t = [x_t x_t(index) + Ts*v*cos(th_t(index))];
            y_t = [y_t y_t(index) + Ts*v*sin(th_t(index))];
            th_t = [th_t th_t(index) + Ts*w];

            new_error = norm(wp(:,i)-[x_t(end) y_t(end)].');

            if i==size(wp,2)
                if new_error > error % integration brings me after the point, making the algorithm believe I am far from it (MEGATRICK)
                   pos_ok=true; 
                end
            end
            
            error=new_error;

            if i<size(wp,2)-1 %not one before last wp
                error_next = norm(wp(:,i+1)-[x_t(end) y_t(end)].');
                if error_next<error %jump to the next wp if I am near enough (avoid turnback)
                   i=i+1;
                   error = error_next;
                end
                pos_ok = error<threshold;
            else
                if i==size(wp,2) % last wp
                    threshold=1e-3; %I want to be more precise than before
                    if error<threshold
                        pos_ok=true; %the final position has been achieved
                    end
                else
                    pos_ok = error<threshold;
                end
            end
            
            %%%%%%%%%%%%%%%%%%%%%%%%
            %%% COMPUTE CONTROLS %%%
            %%%%%%%%%%%%%%%%%%%%%%%%
            
            % - w
            
            if ~pos_ok %if my position is >= the last one i keep going straight, this is to achieve the v_f
                w = K1 * sin( ( th_t(index) - atan2(y_t(index)-wp(2,i),x_t(index)-wp(1,i)) ) );

                w = w_sat.evaluate(w);
            else
                w=0;
            end

            % - v

            if i<size(wp,2) %not last wp
                % general case, v depending on w
                if w>0 % fixing right wheel velocity to the max
                    v = ( v + v_max-b*w ) / 2;
                else % fixing left wheel velocity to the max
                    v = ( v + v_max+b*w ) / 2;
                end
            else
                %last wp
                if error < f_threshold %If I am close enough I want to converge to v_f
                    alpha = abs(error-threshold) / f_threshold;
                    v=(alpha)*v_max + (1 - alpha)*v_f;
                    if norm(v-v_f)<1e-3;
                        vel_ok = true;
                    end
                end
            end

            v = v_sat.evaluate(v);
            
            %%%%%%%%%%%%%%%%%%%%%%
            
            index = index +1;
            
            if index > 10000
                if verbose
                    cprintf('*[1,0,0]*','!! Excedeed maximum number of itereations !!\n');
                end
                retval=0;
                break
            end
        end
        
        if retval==0
            if verbose
                cprintf('*[1,0,0]*','!! Error !!\n');
            end
            break;
        end
        
end

if debug
    plot(wp(1,:),wp(2,:),'og')
    plot(x_t,y_t)
    axis equal
    legend('start point','end point','control points','nominal trajectory','way points','computed reference trajectory','location','best')
end

if debug
    subplot(1,3,2)
    plot((0:numel(speed(1,:))-1)*Ts,speed(1,:)), title('Linear Velocity')
    subplot(1,3,3)
    plot((0:numel(speed(2,:))-1)*Ts,speed(2,:)), title('Angular Velocity')
end

pos=[x_t(1:end-1); y_t(1:end-1); th_t(1:end-1)];
%q
time=[Ts*(1:index)];

if index <= 10000 
    cost=index*Ts;
end

retval=0; % TODO set all variables so the algorithm can go on

if verbose
    disp([' - Planned in ' num2str(index) ' steps.'])
    disp([' - Cost is ' num2str(cost) ' steps.'])
    disp([' - Execution time is ' num2str(time(end))])
    toc
end

if debug
    pos_ok
    vel_ok
    retval
    i
    error
    pos(:,end)
    keyboard
end

end