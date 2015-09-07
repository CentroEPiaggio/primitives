% setup
% cart_width  = 2;
% cart_height = 0.4;
assi_x = [-10 10]*1;
assi_y = [-1 5];
%
x_cart  = rt_q(:,1);
theta   = rt_q(:,2);
d       = rt_q(:,3);
px = x_cart + d.*cos(theta);
py = d.*sin(theta);
% plot routine
figure
set( gcf, 'DoubleBuffer', 'on' );
for ii=1:10:length(rt_t)
    clf
    % cart
    X_cart = [x_cart(ii)-cart_width/2;x_cart(ii)+cart_width/2;x_cart(ii)+cart_width/2;x_cart(ii)-cart_width/2];
    Y_cart = [-cart_height;-cart_height;0;0];  
    patch(X_cart,Y_cart,'r');
    hold on
    % pendulum
    X_pend = [x_cart(ii);px(ii)];
    Y_pend = [0;py(ii)];
    patch(X_pend,Y_pend,'b');
    plot(px(ii),py(ii),'o','linewidth',4);
    if rt_zmpflag(ii) == 0
        colorzmp = 'g';
    else
        colorzmp = 'r';
    end
    plot(rt_xzmp(ii),-cart_height,[colorzmp 'o'],'linewidth',4);
     
    axis([-15 15 -1 5]);
    
    text(x_cart(ii)+cart_width/2,cart_height/2,[num2str(rt_t(ii)) '/' num2str(rt_t(end))]); % clock
    drawnow;
    pause(0.001);
end