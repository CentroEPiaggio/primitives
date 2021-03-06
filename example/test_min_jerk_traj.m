clear all;clc;
close all;

Ts = 0.01;
V = 0.5;
A = 0.1;
D = 0.1;
J = 1;
state_bounds = [Inf Inf; -V V; -D A]*1;
control_bounds = [-J; J]*1;

n = 3;

results_retval = zeros(n,n,n);
results_errnorm = zeros(n,n,n);

zz=1;

p0vec = linspace(-n,n,n+1);
v0vec = linspace(-n,n,n+1);
vfvec = linspace(-n,n,n+1);

for ii=0:n
    for jj=0:n
        for kk=0:n
            disp([num2str(1+ii+jj+kk) '/' num2str((n+1)^3)]);
            
            v0 = v0vec(ii+1);
            vf = vfvec(jj+1);
            pf = p0vec(kk+1);
            
            if abs(v0) > V
                v0 = V;
            end
            if abs(vf) > V
                vf = V;
            end
            
            x0 = [0;v0;0];
            xf = [pf;vf;0];
            
%             % in this case the solution should be extended with a further
%             % T4>0 part
%             x0 = [0;0;0];
%             xf = [0.5;0.5;0];
%             
% %             % 
% %             x0 = [0;0.5;0];
% %             xf = [1;0;0];
% 
%             x0 = [0;0.5;0];
%             xf = [2;0;0];

%             x0 = [0;0.5;0];
%             xf = [1;0.5;0];

% discontinuita' su T3-T4
%              x0 = [0;0;0];
%              xf = [3;0;0];

            if isequal(x0,xf)
                results_retval(ii+1,jj+1,kk+1) = -100;
                continue;
            end
%             x0 = [0;0.4;0];
%             xf = [0.3;0.6;0];
%% TODO: THIS CASE FAILS!
vmax = 1;
amax = 0.5;
jerkmax = 1;
% [time,traj_vel_cart] = trajectory_ks(xi,xf,xpi,xpf,Tend,Ts,amax);

state_bounds = [-Inf Inf;
    -vmax vmax;
    -amax amax];
control_bounds = [-jerkmax;jerkmax];

% also this one!
% x0 = [2.6600; 0.4000; 0];
% xf = [2.4300; 0.5000; 0];
% here is a nice collection of suspected wrong test points
% wrong_points = importdata('log_move_wrong.txt');

                    
x0 =[ 1.7000;     0.7700; 0];
xf = [1.8100; 
    0.7300; 0];
            [time_an,pos_an,speed_an,acc_an,jerk_an,retval] = min_jerk_trajectory_analytic(x0,xf,Ts,state_bounds,control_bounds);
return
            if retval == 1
                x0_res = [pos_an(1);speed_an(1);jerk_an(1)];
                xf_res = [pos_an(end);speed_an(end);jerk_an(end)];
                
                if mod(zz,10)==0
                    figure
                    zz=1;
%                     return
                end
                subplot(3,3,zz);
                zz=zz+1;
                plot_traj(time_an,pos_an,speed_an,acc_an,jerk_an,x0,xf,Ts,A,D,V,J);
                results_errnorm(ii+1,jj+1,kk+1) = norm(x0-x0_res);
%                 return
            else
                keyboard
            end
            results_retval(ii+1,jj+1,kk+1) = retval;
%             return
        end
    end
end

%%
% GRID = meshgrid(0:n,0:n,0:n);
% mesh(GRID,results_retval)
% %%
% x0 = -x0;
% xf = -xf;
%%
figure
spy(results_retval>0)
title(['Total testcases: ' num2str(n*n*n)])
return
%%
% J=J/10;
acc = cumtrapz(time_an,jerk_an);
speed = x0(2)+cumtrapz(time_an,acc);
pos = x0(1)+cumtrapz(time_an,speed);
time = time_an;
pf = xf(1);
figure
plot(time,pos,'b',time,speed,'r',time,acc,'k',time,jerk_an,'c','linewidth',2),grid on,legend('pos','vel','acc','control jerk','location','best')
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
return
[time,pos,speed,acc,jerk] = min_jerk_trajectory(x0,xf,Ts,state_bounds,control_bounds,time_an(end),jerk_an)
close all
plot(time,pos,'b',time,speed,'r',time,acc,'k',time,jerk,'c','linewidth',2),grid on,legend('pos','vel','acc','control jerk','location','best')
hold on
plot(time(1),x0(1),'bo',time(end),xf(1),'bo','linewidth',2)
plot(time(1),x0(2),'ro',time(end),xf(2),'ro','linewidth',2)
plot(time(1),x0(3),'ko',time(end),xf(3),'ko','linewidth',2)

return
% min_jerk_trajectory_nopos(x0,xf,Ts,state_bounds,control_bounds)
[x,v,a,t] = profilo_trapezoidale(x0(1),xf(1),state_bounds(2,2),state_bounds(3,2),control_bounds(2),Ts);
figure
subplot(3,1,1)
plot(time,pos,t,x),grid on
subplot(3,1,2)
plot(time,speed,t,v),grid on
subplot(3,1,3)
plot(time,acc,t,a),grid on