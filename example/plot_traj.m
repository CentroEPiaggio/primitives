function plot_traj(time_an,pos_an,speed_an,acc_an,jerk_an,x0,xf,Ts,A,D,V,J)
% acc = cumtrapz(time_an,jerk_an);
% speed = x0(2)+cumtrapz(time_an,acc);
% pos = x0(1)+cumtrapz(time_an,speed);
% pos = 3*x0(1)+pos_an;
% speed = 2*x0(2)+speed_an;
% speed = 2*speed_an(1)+speed_an;
time = time_an;
pf = xf(1);
% figure
plot(time,pos_an,'b',time,speed_an,'r',time,acc_an,'k',time,jerk_an,'c','linewidth',2),grid on,legend('pos','vel','acc','control jerk','location','best')
hold on
% plot(time(1),x0(1),'bo',time(end),xf(1),'bo','linewidth',2)
plot(time(1),x0(1),'bo',time(end),pf,'bx','linewidth',2)
plot(time(1),x0(2),'ro',time(end),xf(2),'ro','linewidth',2)
plot([time(1),time(end)],[V V],'r-.');
plot([time(1),time(end)],[-V -V],'r-.');
plot(time(1),x0(3),'ko',time(end),xf(3),'ko','linewidth',2)
plot([time(1),time(end)],[A A],'k--');
plot([time(1),time(end)],[-D -D],'k--');
plot([time(1),time(end)],[J J],'c--');
plot([time(1),time(end)],[-J -J],'c--');
title('Trajectory')
end
