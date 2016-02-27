
function [x,y,z,A_tot,A] = DK_arm(q)

%Parameters theta(Rotations around axes of Joint) %radians
theta=q;
%Forward Kinematics
%Lenght of the arms of the serial chain are: 90 mm, 110 mm , 110 mm.

%Parameters a (Convention D-H)
a = [0,0.09,0.11,0.11]; 

%Parameters d
d = [0,0,0,0];

%Parameter alpha(Rotations around axes x-local of the Frames)
alpha = [90,0,0,0];


%definizioni di matrici di rototraslazione relative fra i vari giunti
A_tot = eye(4);



for i = 1:4
   	 %rototranslations-matrix from i-1 to i-th joint
A(:,:,i) = [ cos(theta(i)), -sin(theta(i))*cosd(alpha(i)), sin(theta(i))*sind(alpha(i)), a(i)*cos(theta(i));
             sin(theta(i)), cos(theta(i))*cosd(alpha(i)), -cos(theta(i))*sind(alpha(i)), a(i)*sin(theta(i));
	         0, sind(alpha(i)), cosd(alpha(i)), d(i);
	         0, 0, 0, 1];     	
end
	  
%compute of the total rototranslations-matrix	
A_tot = A(:,:,1)*A(:,:,2)*A(:,:,3)*A(:,:,4);
x=A_tot(1,4);
y=A_tot(2,4);
z=A_tot(3,4);


