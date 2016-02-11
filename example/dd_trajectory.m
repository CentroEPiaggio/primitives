function [time,pos,speed,acc,jerk,retval,cost] = dd_trajectory(x0,xf,Ts,state_bounds,control_bounds)
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

cost=Inf;

x_i=x0(1);
y_i=x0(2);
th_i=x0(3);
v_i=x0(4);

x_f=xf(1);
y_f=xf(2);
th_f=xf(3);
v_f=xf(4);

% B-spline calculation

% the trick of the century: I add a point after the start, and a point
% before the end, to enforce spline starting and ending orientation

P = [x_i x_i+0.1*cos(th_i) x_f-0.1*cos(th_f) x_f; 
     y_i y_i+0.1*sin(th_i) y_f-0.1*sin(th_f) y_f]

c_u(1,:) = linspace(x_i,x_f,100);
c_u(2,:) = interp1(P(1,:),P(2,:),c_u(1,:),'spline');

hold on
set (gcf, 'Units', 'normalized', 'Position', [0,0,1,1]);
plot(P(1,1:2),P(2,1:2),'or')
plot(P(1,3:4),P(2,3:4),'or')
plot(c_u(1,:),c_u(2,:))
axis equal

% trajectory following

error = 0;
threshold = 0.1;

x_t=[];
x_t = [x_t x0(1)];
y_t=[];
y_t = [y_t x0(2)];
th_t=[];
th_t = [th_t x0(3)];
index = 1;

while error>threshold
    v=1;
    w= sin( ( th_t(index) - atan2(y_t(index)-y_f,x_t(index)-x_f) ) );

    % integration
    % x(t) = x0 + int[0,T]{ v cos(th) }
    % y(t) = y0 + int[0,T]{ v sin(th) }
    % th(t) = th0 + int[0,T]{ w }
    
    x_t = [x_t x_t(index) + Ts*v*cos(th_t(index))];
    y_t = [y_t y_t(index) + Ts*v*sin(th_t(index))];
    th_t = [th_t th_t(index) + Ts*w];

    error = norm(xf(1:2)-[x_t(end) y_t(end)].')
    index = index +1;
end

plot(x_t,y_t)
axis equal

retval=1;
end