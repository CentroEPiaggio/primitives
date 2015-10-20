% setup
cart_width  = 2;
cart_height = 0.4;
assi_x = [-10 10]*1;
assi_y = [-1 5];
%
x_cart  = rt_q(:,1);
theta   = rt_q(:,2);
d       = rt_q(:,3);
px = x_cart + d.*cos(theta);
py = d.*sin(theta);
% plot routine
  fh=findall(0,'type','figure');
  fnum = length(fh)+1;
figure(fnum)
set( gcf, 'DoubleBuffer', 'on' );

movie=1;

if movie==1
    frames=1;
    vidObj=VideoWriter('anima.avi');
    vidObj.Quality = 50;
    vidObj.FrameRate = 100;
    open(vidObj);
end

for ii=1:10:length(rt_t)
    figure(fnum)
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
    plot(rt_com(ii,1),rt_com(ii,3),'k+');
    if rt_zmpflag(ii) == 0
        colorzmp = 'g';
    else
        colorzmp = 'r';
    end
    plot(rt_xzmp(ii),-cart_height,[colorzmp 'o'],'linewidth',4);
    
    axis([-5 50 -1 5]);
    
    text(x_cart(ii)+cart_width/2,cart_height/2,[num2str(rt_t(ii)) '/' num2str(rt_t(end))]); % clock
    drawnow;
    pause(0.001);
    
    if(movie==1)
        movie_anima(frames) = getframe(figure(fnum));
        frames = frames +1;
    end
    
end

if(movie==1)
   disp('saving anima video...');
   
   for iter=1:10:frames-1
       vidObj.writeVideo(movie_anima(iter));
   end
   
   close(vidObj);
   
   disp('...done!');
end