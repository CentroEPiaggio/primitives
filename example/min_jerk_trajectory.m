function [time,speed] = min_jerk_trajectory(x0,xf,Ts,state_bounds,control_bounds)
% ensure vectors are column
x0 = x0(:);
xf = xf(:);
control_bounds = control_bounds(:);
% default time step
Ts = 0.1; % HARDCODED for speeding up execution time
if nargin ~= 4
    Ts = 0.1;
end

A = [0 1 0;
    0 0 1;
    0 0 0];
B = [0; 0; 1];
C = [1 0 0;
    0 1 0];
D = zeros(size(C,1),size(B,2));
B_u = B;

m = size(B,2);
n = size(A,1);

% % Eulero in avanti
% Ad = A*Ts + eye(size(A));
% Bd = B*Ts;

Ad = A*Ts+eye(size(A));
Bd = Ts*B;
Cd = C;


% ZOH
M = expm([A,B_u; zeros(m,n+m)]*Ts);
Ad = M(1:n,1:n);
Bd = M(1:n,n+1:n+m);
Cd = C;
Dd = D;

%% test reachability

Rd = ctrb(Ad,Bd);

% if (rank(Rd) < n)
%     warning('The discrete system is NOT completely reachable!!!');
% else
%     disp('Discrete system completely reachable');
% end

%% optimal control

% number of steps - at least 3 (4 to keep constant final value)
steps = 3;
Ttot = steps*Ts;

% reachability matrix in p steps
Rp = Bd;
for i=2:steps
    Rp = [Bd Ad*Rp];
end

% %% optimal unconstrained control
% u = pinv(Rp)*(xf - Ad^steps*x0);
% 
% % % input for simulink
% % uSim = [Ts*(0:steps-1).', flipud(u)];
% % y_lin = lsim(sys,uSim(:,2),uSim(:,1),x0,'zoh');
% % ySim = [(0:steps-1).'*Ts, y_lin]; %#ok
% 
% %% optimal control keeping final state
% 
% % last control value
up = pinv(B_u)*(-A*xf);
% % check whether this worked
% if (norm(A*xf+B_u*up) < 1e-10)
% 	disp('Equilibrium control correctly found!')
% else
% 	warning('u(p) is NOT an equilibrium control!')
% end
% 
% u = pinv(Ad*Rp)*(xf - Ad^(steps+1)*x0 - Bd*up);
% u = [up; u];
% 
% % % input for simulink
% % uSim = [Ts*(0:steps).', flipud(u)];
% % y_lin = lsim(sys,uSim(:,2),uSim(:,1),x0,'zoh');
% % ySim = [(0:steps).'*Ts, y_lin]; %#ok
% 
% %% optimal control with constraints on u - pinv
% 
% % bound on control variable
% u_lb = -20; % lower bound
% u_ub = +20; % upper bound
% % increase step number until the control is bounded
% while not(and(min(u)>=u_lb,max(u)<=u_ub))
%    steps = steps+1;
%    Rp = [Bd Ad*Rp];
%    u = pinv(Ad*Rp)*(xf - Ad^(steps+1)*x0 - Bd*up);
% end
% 
% u = flipud(u);
% u = [u; up];
% 
% % uSim = [(0:steps).'*Ts, u];
% % y_lin = lsim(sys,uSim(:,2),uSim(:,1),x0,'zoh');
% % ySim = [(0:steps).'*Ts, y_lin]; %#ok
% 
% Ttot = steps*Ts;
% disp(['Il tempo minimo ottenuto e'' pari a ' num2str(Ttot)]);

%% optimal control with constraints on u - lsqlin
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
    error('Error in state constraint specification');
end

% keyboard

steps = 3;
% steps = 93+19;
% reachability matrix in p steps
Rp = Bd;
for i=2:steps
    Rp = [Bd Ad*Rp];
end
% keyboard
% initialize flag to a non-acceptable value
exitflag = 0;
% cycle until constraints are satisfied
optiontslsqlin = optimset('display','off','LargeScale','off');
steps_increment = max(round(0.1/Ts),1); % increments of 0.1 seconds


while(exitflag ~= 1)
    steps = steps + steps_increment;
    for ii=1:steps_increment
        Rp = [Bd Ad*Rp];
    end
    
    Ccost = eye(steps);
    d_cost = zeros(steps,1);
    % state constraints
    A_StateBound = zeros(n*steps,steps);
b_StateBound = [];
for ii=1:steps
    A_StateBound = A_StateBound + fliplr(kron(diag(ones(steps-ii+1,1),-(ii-1)),Ad^(ii-1)*Bd));
    b_StateBound = [Ad^ii; b_StateBound];
end
b_StateBound = b_StateBound*x0;
    lower_state = repmat(lower_state_bound,steps,1);
    upper_state = repmat(upper_state_bound,steps,1);
    H_matrix = [A_StateBound;
        -A_StateBound];
    O_matrix = [upper_state - b_StateBound;
        -lower_state + b_StateBound];
    % to keep final state
    Aeq = Ad*Rp;
    beq = xf - Ad^(steps+1)*x0 - Bd*up;

    % lower and upper bounds
    lb = lower_control_bound*ones(steps,1);
    ub = upper_control_bound*ones(steps,1);
% steps*Ts
    % solution with lsqlin
    
    [u,~,~,exitflag] = lsqlin(Ccost,d_cost,H_matrix,O_matrix,Aeq,beq,lb,ub,[],optiontslsqlin);
%     [u,~,~,exitflag] = lsqlin(Ccost,d_cost,[],[],Aeq,beq,lb,ub,[],optiontslsqlin);
end

u = flipud(u);
u = [u; up];

Ttot = steps*Ts;
disp(['Il tempo minimo ottenuto e'' pari a ' num2str(Ttot)]);
% extract speed trajectory
time_largesampling=0:Ts:Ttot;
Cextract = [0 1 0];
states = zeros(3,steps+1);
states(:,1) = x0;
for ii=2:steps+1
    states(:,ii)=Ad*states(:,ii-1) + Bd*u(ii-1);
end
keyboard
% figure, plot(time_largesampling,states)
speed = Cextract*states;
time = 0:0.01:Ttot;
speed = interp1(time_largesampling,speed,time);