close all;

figure
subplot(3,1,1);
plot(t,q(:,1),t,q_ref(:,1));
legend('x\_cart','x\_cart\_ref')
subplot(3,1,2);
plot(t,q(:,2),t,q_ref(:,2));
legend('\theta','\theta\_ref')
subplot(3,1,3);
plot(t,q(:,3),t,q_ref(:,3));
legend('l','l\_ref')