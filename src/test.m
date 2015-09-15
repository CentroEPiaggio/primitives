%%
load modello.mat;
m1 = 100;
m2 = 1;
m3 = 100;
masses=[m1;m2;m3];
p1x = rt_q(:,1);
p2x = p1x;
 p2x = p1x;      % POSIZIONI DEI GIUNTI DURANTE la primitiva MUOVI
    p3x = p1x;      % POSIZIONI DEI GIUNTI DURANTE la primitiva MUOVI
    px = [p1x,p2x,p3x];
    p1z = 0; p2z = 0; p3z= 2; % altezza COSTANTE dei giunti durante la primitiva MUOVI
    pz = [p1z,p2z,p3z];
    amax_upper = (p1x+1 - (masses(1)*p1x + masses(2)*p2x + masses(3)*p3x)/sum(masses)) * (sum(masses)*9.8)/(3*dot(masses,pz));
    amax_lower = (p1x-1 - (masses(1)*p1x + masses(2)*p2x + masses(3)*p3x)/sum(masses)) * (sum(masses)*9.8)/(3*dot(masses,pz));
    amax_desired = min(max(abs(amax_upper)),max(abs(amax_lower)));
    disp(['amax_desired: ' num2str(amax_desired)])
    
    acc_x = gradient(rt_qp(:,1))/mean(diff(rt_t));
    xzmp = (masses(1)*p1x + masses(2)*p2x + masses(3)*p3x)/sum(masses)+3*acc_x*dot(masses,pz)/(sum(masses)*9.8);
    figure
    plot(rt_t,xzmp,rt_t,rt_xzmp)
    hold on
    plot([0 rt_t(end)],[amax_desired amax_desired],'r')
    plot([0 rt_t(end)],-[amax_desired amax_desired],'r')
    legend('teorico','vero','amax\_desired','location','best')
    grid on