x_i = 0;
y_i = 0;
th_i = 0;
x_f = 1;
y_f = 1;
th_f = 0;

control_factor = 0.7;

for th_f=0:0.1:2*pi

    P = [x_i x_i+control_factor*cos(th_i) x_f-control_factor*cos(th_f) x_f; 
         y_i y_i+control_factor*sin(th_i) y_f-control_factor*sin(th_f) y_f];

    pt1 = P(:,1);
    pt2 = P(:,2);
    pt3 = P(:,3);
    pt4 = P(:,4);

    t = linspace(0,1,101);
    c_u = kron((1-t).^3,pt1) + kron(3*(1-t).^2.*t,pt2) + kron(3*(1-t).*t.^2,pt3) + kron(t.^3,pt4);
    hold on
    set (gcf, 'Units', 'normalized', 'Position', [0,0,1,1]);
    plot(P(1,:),P(2,:),'or')
    plot(c_u(1,:),c_u(2,:))
    axis equal

end
% trajectory following