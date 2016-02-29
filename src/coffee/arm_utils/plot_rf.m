% plot a reference frame
function plot_rf(A,str)
versor_length = 0.1;
ob = A(1:3,4);
axis_vector = [ob'; ob'; ob'];
R = versor_length*A(1:3,1:3)';

% quiver3(axis_vector(:,1),axis_vector(:,2),axis_vector(:,3),[versor_length;0;0],[0;versor_length;0],[0;0;versor_length])
quiver3(axis_vector(:,1),axis_vector(:,2),axis_vector(:,3),R(:,1),R(:,2),R(:,3))
if nargin == 2
    text(axis_vector(1,1)-0.02,axis_vector(2,2)-0.02,axis_vector(3,3)-0.02,str);
end
end

% % plot a reference frame
% function plot_rf(ob)
% versor_length = 1;
% versor = diag(versor_length*ones(3,1));
% origin = [ob(:),ob(:),ob(:)];
% % P = A(1:3,4) + A(1:3,1:3)*versor;
% P = eye(3);
% quiver3(origin(:,1),origin(:,2),origin(:,3),P(:,1),P(:,2),P(:,3))
% % if nargin == 2
% %     text(A(1,1)-0.02,A(2,2)-0.02,A(3,3)-0.02,str);
% % end
% xlabel('x')
% ylabel('y')
% zlabel('z')
% end