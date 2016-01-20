function [time,pos,speed,acc,jerk,retval] = min_jerk_trajectory_analytic(x0,xf,Ts,state_bounds,control_bounds)
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
p0 = x0(1);
v0 = x0(2);
pf = xf(1);
vf = xf(2);
A = abs(upper_state_bound(3))
D = abs(lower_state_bound(3))
V = upper_state_bound(2)
J = upper_control_bound
L = xf(1)-x0(1)

if abs(v0) > V || abs(vf) > V
    disp('Bounds are not consistent with the problem: initial and/or final speed outside constraints');
    retval = -1;
    return
end

%% determine if AFP or DFP: Acceleration or Deceleration First Profile
if v0<=vf && vf-v0 <= A^2/J
    Tm = 2*sqrt((vf-v0)/J);
elseif v0<=vf && vf-v0 > A^2/J
    Tm = (vf-v0)/A + A/J;
elseif v0>vf && v0-vf <= D^2/J
    Tm = 2*sqrt((v0-vf)/J);
elseif v0>vf && v0-vf > D^2/J
    Tm = (v0-vf)/D + D/J;
else
    disp('could not determine if acceleration profile is AFP or DFP');
    retval = -3;
    return
end

Lm = (v0+vf)/2*Tm;
if (v0 <= vf && Lm <= L) || (v0>vf && Lm < L)
    AFP = 1;
    disp('AFP');
elseif (v0 <= vf && Lm > L) || (v0 > vf && Lm >= L)
    AFP = 0;
    disp('DFP');
else
    disp('could not determine if acceleration profile is AFP or DFP');
    retval = -4;
    return
end
if debug,keyboard,end
%% rearrange problem in AFP form
if ~AFP % if DFP
    p0 = -p0;
    pf = -pf;
    v0 = -v0;
    vf = -vf;
    temp = D;
    D = A;
    A = temp;
end

%% case 1: T4>0, T2>0, T6>0
if verbose
    disp('attempt with case 1: T4>0, T2>0, T6>0');
end
if debug,keyboard,end
x=[];xbar=[];xhat=[];
x = (V-v0)/A + A/J;
xbar = (V-vf)/D + D/J;
xhat = (2*L-(v0+V)*x - (V+vf)*xbar)/(2*V);
if x>=2*A/J && xbar>=2*D/J && xhat>=0
    Times = getTimes(A,D,J,x,xhat,xbar);
    if Times(4)>0 && Times(2)>0 && Times(6)>0
        disp('case 1: T4>0, T2>0, T6>0')
        tvec = Times;
        vp = V;
        found = 1;
        if debug,keyboard,end
    end
end
%% case 2: T4>0, T2=0, T6>0
if verbose
    disp('attempt with case 2: T4>0, T2=0, T6>0');
end
if ~found
    x=[];xbar=[];xhat=[];
    x = 2*sqrt((V-v0)/J);
    xbar = (V-vf)/D + D/J;
    xhat = (2*L-(v0+V)*x - (V+vf)*xbar)/(2*V);
    if x>=0 && x<=2*A/J && xbar>=2*D/J && xhat>=0
        % adjust boundaries when the constant acceleration or deceleration phase do not exist
        A_tmp = A;
        vp = V;
        if eps+(vp-v0) >= 0.25*J*x^2 % numerical issues, added eps to force solution in cases where it is true
            A_tmp = 0.5*J*x;
        end
        
        Times = getTimes(A,D,J,x,xhat,xbar);
        if Times(4)>0 && Times(2)==0 && Times(6)>0
            disp('case 2: T4>0, T2=0, T6>0')
            tvec = Times;
            vp = V;
            found=1;
        end
    end
end
%% case 3: T4>0, T2>0, T6=0
if verbose
    disp('attempt with case 3: T4>0, T2>0, T6=0');
end
if ~found
    x=[];xbar=[];xhat=[];
    x = (V-v0)/A + A/J;
    xbar = 2*sqrt((V-vf)/J);
    xhat = (2*L-(v0+V)*x - (V+vf)*xbar)/(2*V);
    if x>=2*A/J && xbar>=0 && xbar<=2*D/J && xhat>=0
        % adjust boundaries when the constant acceleration or deceleration phase do not exist
        D_tmp = D;
        vp = V;
        if eps+(vp-vf) >= 0.25*J*xbar^2 % numerical issues, added eps to force solution in cases where it is true
            D_tmp = 0.5*J*xbar;
        end
        
        Times = getTimes(A,D_tmp,J,x,xhat,xbar);
        if Times(4)>0 && Times(2)>0 && Times(6)==0
            disp('case 3: T4>0, T2>0, T6=0')
            tvec = Times;
            vp = V;
            found = 1;
        end
    end
end
%% case 4: T4>0, T2=0, T6=0
if verbose
    disp('attempt with case 4: T4>0, T2=0, T6=0');
end
if ~found
    x=[];xbar=[];xhat=[];
    x = 2*sqrt((V-v0)/J);
    xbar = 2*sqrt((V-vf)/J);
    xhat = (2*L-(v0+V)*x - (V+vf)*xbar)/(2*V);
    if x>=0 && x<=2*A/J && xbar>=0 && xbar<=2*D/J && xhat>=0
        % adjust boundaries when the constant acceleration or deceleration phase do not exist
        A_tmp = A;
        D_tmp = D;
        vp = V;
        if eps+(vp-v0) >= 0.25*J*x^2 % numerical issues, added eps to force solution in cases where it is true
            A_tmp = 0.5*J*x;
        end
        if eps+(vp-vf) >= 0.25*J*xbar^2 % numerical issues, added eps to force solution in cases where it is true
            D_tmp = 0.5*J*xbar;
        end
        
        Times = getTimes(A_tmp,D_tmp,J,x,xhat,xbar);
        if Times(4)>0 && Times(2)==0 && Times(6)==0
            disp('case 4: T4>0, T2=0, T6=0')
            tvec = Times;
            vp = V;
            found = 1;
        end
    end
end
%% if T4=0, xhat=0;
if ~found
    xhat = 0;
end
if debug,keyboard,end
%% case 5: T4=0, T2=0, T6=0
if verbose
    disp('attempt with case 5: T4=0, T2=0, T6=0');
end
if ~found
    x=[];xbar=[];
    coeffs = [1/4*(vf-v0)*J, J*L, -(vf-v0)^2, +8*v0*L, -4*(L^2+1/J*(v0+vf)^2*(vf-v0))];
    sol = roots(coeffs);
    jj=1;
    x = [];
    for ii=1:length(sol)
        if isreal(sol(ii))
            if sol(ii) >=0 && sol(ii)<=2*A/J % condition on x
                x(jj) = sol(ii);
                jj=jj+1;
            end
        end
    end
    if ~isempty(x)
%         if length(x)>1
%             keyboard
%         end
        vp = v0+1/4*J*x.^2;
        %         coeffs = [1/4*J, 0, vf-vp]; % TODO: check dimensional issues
        coeffs = [1/4*J*ones(length(vp),1), zeros(length(vp),1), vf-vp(:)]; % TODO: check dimensional issues
        for kk=1:length(vp)
            sol = roots(coeffs(kk,:));
            jj=1;
            xbar = [];
            for ii=1:length(sol)
                if isreal(sol(ii))
                    if sol(ii) >=0 && sol(ii)<=2*D/J % condition on x
                        xbar(jj,kk) = sol(ii);
                        jj=jj+1;
                    end
                end
            end
        end
    end
    xbar = xbar(xbar>0);
    if ~isempty(xbar)
        % adjust boundaries when the constant acceleration or deceleration phase do not exist
        A_tmp = A;
        D_tmp = D;
        if eps+(vp-v0) >= 0.25*J*x^2 % numerical issues, added eps to force solution in cases where it is true
            A_tmp = 0.5*J*x;
        end
        if eps+(vp-vf) >= 0.25*J*xbar^2 % numerical issues, added eps to force solution in cases where it is true
            D_tmp = 0.5*J*xbar;
        end
        
        Times = getTimes(A_tmp,D_tmp,J,x,xhat,xbar);
        if Times(4)==0 && Times(2)==0 && Times(6)==0
            disp('case 5: T4==0, T2=0, T6=0');
            tvec = Times;
            found = 1;
        end
    end
end
%% case 6: T4=0, T2>0, T6=0
if verbose
    disp('attempt with case 6: T4=0, T2>0, T6=0');
end
if ~found
    x=[];xbar=[];
    coeffs = [J^2/(16*A), 1/4*J, 1/4*(2*(J*vf/A)+A), 2*vf, -2*L+1/A*(vf+v0)*(vf-v0+A^2/J)];
    sol = roots(coeffs);
    jj=1;
    xbar = [];
    for ii=1:length(sol)
        if isreal(sol(ii))
            if sol(ii) >=0 && sol(ii)<=2*D/J % condition on xbar
                xbar(jj) = sol(ii);
                jj=jj+1;
            end
        end
    end
    if ~isempty(xbar)
        %         if length(xbar)>1
        %             keyboard
        %         end
        vp = vf+1/4*J*xbar.^2;
        x = (vp-v0+A^2/J)/A;
    end
    x = x(x>= 2*A/J);
    if ~isempty(x)
        % adjust boundaries when the constant acceleration or deceleration phase do not exist
        D_tmp = D;
        if eps+(vp-vf) >= 0.25*J*xbar^2 % numerical issues, added eps to force solution in cases where it is true
            D_tmp = 0.5*J*xbar;
        end
        Times = getTimes(A,D_tmp,J,x,xhat,xbar);
        
        if Times(4)==0 && Times(2)>0 && Times(6)==0
            disp('case 6: T4=0, T2>0, T6=0');
            tvec = Times;
            found = 1;
        end
        
    end
end

%% case 7: T4=0, T2=0, T6>0
if verbose
    disp('attempt with case 7: T4=0, T2=0, T6>0');
end
if ~found
    x=[];xbar=[];
    coeffs = [J^2/(16*D), 1/4*J, 1/4*(2*J*v0/D+D), 2*v0, -2*L+1/D*(v0+vf)*(v0-vf+D^2/J)];
    sol = roots(coeffs);
    jj=1;
    x = [];
    for ii=1:length(sol)
        if isreal(sol(ii))
            if sol(ii) >=0 && sol(ii)<=2*A/J % condition on x
                x(jj) = sol(ii);
                jj=jj+1;
            end
        end
    end
    if ~isempty(x)
        %         if length(x)>1
        %             keyboard
        %         end
        vp = v0+1/4*J*x.^2;
        xbar = (vp-vf+D^2/J)/D;
    end
    xbar = xbar(xbar>= 2*D/J);
    if ~isempty(xbar)
        % adjust boundaries when the constant acceleration or deceleration phase do not exist
        A_tmp = A;
        if eps+(vp-v0) >= 0.25*J*x^2 % numerical issues, added eps to force solution in cases where it is true
            A_tmp = 0.5*J*x;
        end
        Times = getTimes(A_tmp,D,J,x,xhat,xbar);
        if Times(4)==0 && Times(2)==0 && Times(6)>0
            disp('case 7: T4=0, T2=0, T6>0');
            tvec = Times;
            found = 1;
        end
    end
end
%% case 8: T4=0, T2>0, T6>0
if verbose
    disp('attempt with case 8: T4=0, T2>0, T6>0');
end
if debug,keyboard,end
if ~found
    x=[];xbar=[];
    coeffs = [A*(A/D+1), 1/(J*D)*(A+D)*(A*D-2*A^2+2*v0*J), -2*L-1/D*(v0+vf-A^2/J)*(vf-v0+(A^2-D^2)/J)];
    sol = roots(coeffs);
    jj=1;
    x = [];
    for ii=1:length(sol)
        if isreal(sol(ii))
            if sol(ii)>=2*A/J % condition on x
                x(jj) = sol(ii);
                jj=jj+1;
            end
        end
    end
    if ~isempty(x)
%         if length(x)>1
%             keyboard
%         end
        vp = v0-A^2/J+A*x;
        xbar = (vp-vf+D^2/J)/D;
        if length(xbar)>1
            x = x(xbar>=0);
            vp = vp(xbar>=0);
            xbar = xbar(xbar>=0);
        end
    end
    if debug,keyboard,end
    if ~isempty(xbar) && xbar>= 2*D/J
        Times = getTimes(A,D,J,x,xhat,xbar);
        if Times(4)==0 && Times(2)>0 && Times(6)>0
            disp('case 8: T4=0, T2>0, T6>0');
            tvec = Times;
            found = 1;
        end
    end
end
if ~found
    time = [];
    pos = [];
    speed = [];
    acc = [];
    jerk = [];
    disp('Nothing found!');
    retval = 0;
%     keyboard
    return
end
%% Compute times
Tfin = sum(tvec);
t1 = tvec(1);
t2 = t1+tvec(2);
t3 = t2+tvec(3);
t4 = t3+tvec(4);
t5 = t4+tvec(5);
t6 = t5+tvec(6);
t7 = t6+tvec(7);
time  = 0:Ts:Tfin;
jerk  = zeros(size(time));
acc   = zeros(size(time));
speed = zeros(size(time));
pos   = zeros(size(time));
T1 = find(time<t1);
T2 = find(time>=t1 & time<t2);
T3 = find(time>=t2 & time<t3);
T4 = find(time>=t3 & time<t4);
T5 = find(time>=t4 & time<t5);
T6 = find(time>=t5 & time<t6);
T7 = find(time>=t6);

%% readjust limits and recompute times
if debug,keyboard,end
if isempty(T2) || (vp-v0)>J/4*x^2
    A = 0.5*J*x;
end
if isempty(T6) || (vp-vf)>J/4*xbar^2
    D = 0.5*J*xbar;
end
tvec = getTimes(A,D,J,x,xhat,xbar);
Tfin = sum(tvec);
t1 = tvec(1);
t2 = t1+tvec(2);
t3 = t2+tvec(3);
t4 = t3+tvec(4);
t5 = t4+tvec(5);
t6 = t5+tvec(6);
t7 = t6+tvec(7);
time  = 0:Ts:Tfin;
jerk  = zeros(size(time));
acc   = zeros(size(time));
speed = zeros(size(time));
pos   = zeros(size(time));
T1 = find(time<t1);
T2 = find(time>=t1 & time<t2);
T3 = find(time>=t2 & time<t3);
T4 = find(time>=t3 & time<t4);
T5 = find(time>=t4 & time<t5);
T6 = find(time>=t5 & time<t6);
T7 = find(time>=t6);

%% profile
if ~isempty(T1)
tau1 = time(T1)-time(T1(1));
jerk(T1) = J;
acc(T1) = J*tau1;
speed(T1) = v0+0.5*J*tau1.^2;
pos(T1) = p0+v0*tau1+J*tau1.^3/6;

v1 = speed(T1(end));
p1 = pos(T1(end));
else
    v1 = v0;
    p1 = p0;
end

if ~isempty(T2)
    tau2 = time(T2)-time(T2(1));
    acc(T2) = A*ones(length(T2),1);
    speed(T2) = v1+A*tau2;
    pos(T2) = p1+v1*tau2+0.5*A*tau2.^2;
    v2 = speed(T2(end));
    p2 = pos(T2(end));
else
    v2 = v1;
    p2 = p1;
end

if ~isempty(T3)
tau3 = time(T3)-time(T3(1));
jerk(T3) = -J;
acc(T3) = A-J*tau3;
speed(T3) = v2+A*tau3-0.5*J*tau3.^2;
pos(T3) = p2+v2*tau3+0.5*A*tau3.^2-J*tau3.^3/6;
v3 = speed(T3(end));
p3 = pos(T3(end));
else
    v3 = v2;
    p3 = p2;
end

if ~isempty(T4)
    tau4 = time(T4)-time(T4(1));
    acc(T4) = zeros(size(T4));
    speed(T4) = v3*ones(length(T4),1);
    pos(T4) = p3+v3*tau4;
    v4 = speed(T4(end));
    p4 = pos(T4(end));
else
    v4 = v3;
    p4 = p3;
end
% keyboard
if ~isempty(T5)
tau5 = time(T5)-time(T5(1));
jerk(T5) = -J;
acc(T5) = -J*tau5;
speed(T5) = v4-0.5*J*tau5.^2;
pos(T5) = p4+v4*tau5-J*tau5.^3/6;
v5 = speed(T5(end));
p5 = pos(T5(end));
else
    v5=v4;
    p5=p4;
end

if ~isempty(T6)
    tau6 = time(T6)-time(T6(1));
    acc(T6) = -D*ones(length(T6),1);
    speed(T6) = v5-D*tau6;
    pos(T6) = p5+v5*tau6-0.5*D*tau6.^2;
    v6 = speed(T6(end));
    p6 = pos(T6(end));
else
    v6 = v5;
    p6 = p5;
end

if ~isempty(T7)
tau7 = time(T7)-time(T7(1));
jerk(T7) = J;
acc(T7) = -D+J*tau7;
speed(T7) = v6-D*tau7+0.5*J*tau7.^2;
pos(T7) = p6+v6*tau7-0.5*D*tau7.^2+J*tau7.^3/6;
else
    % do nothing
end
%% hard integration
% acc = cumtrapz(time,jerk);
% speed = x0(2)+cumtrapz(time,acc);
% pos = x0(1)+cumtrapz(time,speed);
%% plot
if verbose
    figure
    plot(time,pos,'b',time,speed,'r',time,acc,'k',time,jerk,'c','linewidth',2),grid on,legend('pos','vel','acc','control jerk','location','best')
    hold on
    plot(time(1),x0(1),'bo',time(end),xf(1),'bo','linewidth',2)
    plot(time(1),x0(2),'ro',time(end),xf(2),'ro','linewidth',2)
    plot([time(1),time(end)],[V V],'r--');
    plot([time(1),time(end)],[-V -V],'r--');
    plot(time(1),x0(3),'ko',time(end),xf(3),'ko','linewidth',2)
    plot([time(1),time(end)],[A A],'k--');
    plot([time(1),time(end)],[-D -D],'k--');
    plot([time(1),time(end)],[J J],'c--');
    plot([time(1),time(end)],[-J -J],'c--');
    title('before rearrangement')
end
%% rearrange solution in AFP form
if ~AFP
%     p0 = -p0;
    pf = -pf;
    v0 = -v0;
%     vf = -vf;
    temp = D;
    D = A;
    A = temp;
%     pos = -pos;
    speed = -speed;
    acc = -acc;
    jerk = -jerk;
end
%% plot
if verbose
    figure
    plot(time,pos,'b',time,speed,'r',time,acc,'k',time,jerk,'c','linewidth',2),grid on,legend('pos','vel','acc','control jerk','location','best')
    hold on
    % plot(time(1),x0(1),'bo',time(end),xf(1),'bo','linewidth',2)
    plot(time(1),x0(1),'bo',time(end),pf,'bo','linewidth',2)
    plot(time(1),x0(2),'ro',time(end),xf(2),'ro','linewidth',2)
    plot([time(1),time(end)],[V V],'r--');
    plot([time(1),time(end)],[-V -V],'r--');
    plot(time(1),x0(3),'ko',time(end),xf(3),'ko','linewidth',2)
    plot([time(1),time(end)],[A A],'k--');
    plot([time(1),time(end)],[-D -D],'k--');
    plot([time(1),time(end)],[J J],'c--');
    plot([time(1),time(end)],[-J -J],'c--');
    title('after rearrangement')
end
%%
retval = 1; % success