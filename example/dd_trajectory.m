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

% Using Bezier curves

control_factor = 0.7;

% the trick of the century: I add a control point after the start, and a 
% control point before the end, to enforce spline starting and ending
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
    hold on
    set (gcf, 'Units', 'normalized', 'Position', [0,0,1,1]);
    plot(P(1,1),P(2,1),'*r')
    plot(P(1,end),P(2,end),'*r')
    plot(P(1,2:end-1),P(2,2:end-1),'*b')
    plot(c_u(1,:),c_u(2,:),'Linewidth',2)
    axis equal
end

% trajectory following

threshold = 0.01;

x_t=[];
x_t = [x_t x0(1)];
y_t=[];
y_t = [y_t x0(2)];
th_t=[];
th_t = [th_t x0(3)];

K1=1;
K2=1;

index = 1;
wp=[];
for i=1:numel(t)
    if mod(i,10)==0
        wp = [wp c_u(:,i+1)];
    end
end

w_sat = saturation([-1 1]);
v_sat = saturation([-1 1]);

for i=1:size(wp,2)

        error = norm(wp(:,i)-[x_t(end) y_t(end)].');

        while error>threshold

            w = K1 * sin( ( th_t(index) - atan2(y_t(index)-wp(2,i),x_t(index)-wp(1,i)) ) );
            
            w = w_sat.evaluate(w);
            
            v = 0.1* (1-abs(w)); % TODO enhance this

            v = v_sat.evaluate(v);

            % integration TODO use cumtrapz
            % x(t) = x0 + int[0,T]{ v cos(th) }
            % y(t) = y0 + int[0,T]{ v sin(th) }
            % th(t) = th0 + int[0,T]{ w }

            x_t = [x_t x_t(index) + Ts*v*cos(th_t(index))];
            y_t = [y_t y_t(index) + Ts*v*sin(th_t(index))];
            th_t = [th_t th_t(index) + Ts*w];

            error = norm(wp(:,i)-[x_t(end) y_t(end)].');
            
            if i<size(wp,2)-1
                error_next = norm(wp(:,i+1)-[x_t(end) y_t(end)].'); %porcata
            
                if error_next<error
                   i=i+1; 
                end
            end

            index = index +1;
            
            if index > 10000
                disp(' !! Excedeed maximum number of itereations !! ')
                break
            end
        end
end

if verbose
    disp(['Planned in ' num2str(index) ' steps.'])
    toc
end

if debug
    plot(wp(1,:),wp(2,:),'og')
    plot(x_t,y_t)
    axis equal
end
retval=0;
end