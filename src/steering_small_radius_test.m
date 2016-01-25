x = rand(2,1);
y = rand(2,1)+2;
eta = 1;
alfa = eta/norm(y-x)
z = (1-alfa)*x+(alfa)*y
figure
plot(x(1),x(2),'bx','linewidth',2)
hold on
plot(y(1),y(2),'kx','linewidth',2)
plot(z(1),z(2),'rx','linewidth',2)

raggio = eta;
centro = x(1:2)-raggio;
diameter = 2*raggio;
cerchio = rectangle('position',[centro',diameter,diameter],... % Draw a circle around the nearest neighbors inside the bubble.
                'curvature',[1 1],'EdgeColor','b'); % 'LineStyle',':'
%                 set(cerchio,'visible','on')

grid on
axis equal