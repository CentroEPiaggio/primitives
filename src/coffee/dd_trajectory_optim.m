function [time,x,u,retval,cost] = dd_trajectory(x0,xf,Ts,state_bounds,control_bounds)
debug = 0;
verbose = 0;
x = [];
u = [];

if verbose
    tic
end

time = [];

Tf = 10;
N = 20;
u_1 = sdpvar(1,N);
u_2 = sdpvar(1,N);
x_0 = [x0(:); 0];
x_f = [xf(:); 0];
x_var = sdpvar(5,N);
objective = 0;

cost=Tf;
retval = 1;

Constraints_control = [u_1<=100, u_1>=-100, u_2<=100, u_2>=-100];
Constraints_state = [x_var(:,1) == x_0; x_var(:,end) == x_f];
v_max = 1;
w_max = 1;
Constraints_state = [Constraints_state, -v_max <= x_var(4,:) <= v_max, -w_max <= x_var(5,:) <= w_max];

h = Tf/N; % passo di integrazione
for k = 1:N-1
    funz = [x_var(4,k)*cos(x_var(3,k)); % x dot
        x_var(4,k)*sin(x_var(3,k)); % y dot
        x_var(5,k); % theta dot
        u_1(k); % v dot
        u_2(k)]; % omega dot
    %     Constraints_state = [Constraints_state, x_var(:,k+1) == x_var(:,k) + h*funz];
        Constraints_state = [Constraints_state, x_var(:,k+1) == x_var(:,k) + h*funz];
    objective = objective + norm(u_1(k)) + norm(u_2(k));
end
Global_constraints = [Constraints_control, Constraints_state];
diagnostics = optimize(Global_constraints,objective,sdpsettings('solver','fmincon','fmincon.MaxIter',200,'fmincon.TolFun',0.1));

if diagnostics.problem ~= 0
    retval = 0;
    cost = Inf;
end

u(1,:) = u_1;
u(2,:) = x_var(5,:);
x = x_var(1:4,:);

time = 0:h:h*(N-1);

if debug
    figure(16)
    plot(double(x_var(1,:)),double(x_var(2,:)))
    figure(17)
    subplot(2,1,1)
    plot(time,double(x_var))
    grid on
    legend('x','y','\theta','v','\omega')
    subplot(2,1,2)
    stairs(time,double(u_1))
    hold on
    stairs(time,double(u_2))
    grid on
    legend('u\_1','u\_2')
end

if verbose
    toc
end
end