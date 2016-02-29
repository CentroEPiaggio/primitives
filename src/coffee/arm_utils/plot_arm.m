function plot_arm(q_arm,A_shoulder_inertial)

[~,~,~,A_tot,A] = DK_arm(q_arm);

A_tot = A_shoulder_inertial*A_tot;
A(:,:,1) = A_shoulder_inertial*A(:,:,1);

%Positions of the Joints
X1=A(1,4,1);
Y1=A(2,4,1);
Z1=A(3,4,1);
A2=(A(:,:,1)*A(:,:,2));
X2=A2(1,4);
Y2=A2(2,4);
Z2=A2(3,4);
A3=(A(:,:,1)*A(:,:,2)*A(:,:,3));
X3=A3(1,4);
Y3=A3(2,4);
Z3=A3(3,4);
X4=A_tot(1,4);
Y4=A_tot(2,4);
Z4=A_tot(3,4);
% axis([-0.32 0.32 -0.32 0.32 0 0.32]),grid on
hold on
   
plot3([X1,X2],[Y1,Y2],[Z1,Z2],'b','Linewidth',2);
plot3([X1,X2],[Y1,Y2],[Z1,Z2],'bo');

plot3( [X2,X3],[Y2,Y3],[Z2,Z3],'b','Linewidth',2); 
plot3([X2,X3],[Y2,Y3],[Z2,Z3],'bo');
	
plot3( [X3,X4],[Y3,Y4],[Z3,Z4],'b','Linewidth',2);
plot3([X3,X4],[Y3,Y4],[Z3,Z4],'b*');
