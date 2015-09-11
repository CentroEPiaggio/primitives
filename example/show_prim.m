%% just testing
Tend = 1;
Ts = 0.001;
[time,traj_y_cart,amax,amin] = trajectory(0,1,0,0,Tend/2,Ts);
% % calculate maximum acceleration
% syms a3 a4 a5 t a2;
% f = (6*a3+24*a4*t+60*a5*t^2);
% t_max=solve(f,t)
% pretty(t_max)
% t = t_max(1);
% acc_1 = 2*a2 + 6*a3*t + 12*a4*t^2 + 20*a5*t^3;
% pretty(simplify(acc_1))
% return
%% kinematic scaling of trajectory
maximo = 1;
[time2,traj_y_cart2,amax2,amin2] = trajectory_ks(0,1,0,0,Tend/2,Ts,maximo);
%% plot
figure
subplot(3,2,1)
plot(time,cumtrapz(time,traj_y_cart))
grid on
legend('position')
subplot(3,2,2)
plot(time2,cumtrapz(time2,traj_y_cart2))
grid on
legend('position rescaled')
subplot(3,2,3)
plot(time,traj_y_cart)
grid on
legend('velocity')
subplot(3,2,4)
plot(time2,traj_y_cart2)
grid on
legend('velocity rescaled')
subplot(3,2,5)
plot(time,gradient(traj_y_cart)/mean(diff(time)))
grid on
hold on
line(time,amax*ones(size(time)),'color','red')
line(time,amin*ones(size(time)),'color','black')
legend('acceleration','max','min')
subplot(3,2,6)
plot(time2,gradient(traj_y_cart2)/mean(diff(time2)))
grid on
hold on
line(time2,amax2*ones(size(time2)),'color','magenta')
line(time2,amin2*ones(size(time2)),'color','green')
legend('rescaled acceleration','max','min')
xlabel('time [s]')