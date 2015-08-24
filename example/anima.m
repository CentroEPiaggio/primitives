% setup
cart_width  = 2;
cart_height = 0.4;
assi_x = [-10 10]*1;
assi_y = [-1 5;]
%
x_cart  = q(:,1);
theta   = q(:,2);
d       = q(:,3);
px = x_cart + d.*cos(theta);
py = d.*sin(theta);
% plot routine
figure
set( gcf, 'DoubleBuffer', 'on' );
for ii=1:length(t)
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
    plot(com(ii),0,'yo','linewidth',4);
    xaxis(assi_x);
    yaxis(assi_y);
    drawnow;
    pause(0.1);
end