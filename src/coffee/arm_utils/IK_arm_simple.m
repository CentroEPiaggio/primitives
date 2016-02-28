function [theta,flag]=IK_arm_simple(x,y,z,approach)
clear theta flag
debug = 1;
if debug
    keyboard
end

	a = [0,0.09,0.11,0.11];     %%Parameter A (Convention D-H)
	flag=0;                     %if solution found return flag=0 otherwise flag=1\
    vector_3d = [x;y;z];
    vector_planar_projection = [x;y;0];
    vector_elevation = acos(dot(vector_3d,vector_planar_projection)/(norm(vector_3d)*norm(vector_planar_projection)));
    
	orientazione_terna=rad2deg(vector_elevation);% Orientation of Approach of the End-effector in the Plane X1-Z0 

% compute the angle of the first joint
    theta1=radtodeg(atan2(y,x));

%projection position of the penultimate joint of the serial chain in the X0-Y0 plane
    P_wx=((sqrt(x^(2)+(y^(2))))-a(4)*cosd(orientazione_terna));
    P_wz= z-(a(4)*sind(orientazione_terna));
	
%compute the cosine and sine of the second and third Joints
    c3=(((P_wx)^(2))+((P_wz)^(2))-((a(2))^(2))-((a(3))^(2)))/(2*a(2)*a(3));
    c3 = round(c3*1e8)/1e8; % dirty fix

    if(abs(c3)>1)
        theta=NaN; %no solution found
        if debug
            disp('22')
            keyboard
        end
		flag=1;
        return
    end
    
    s3=sqrt(1-(c3^(2)));
    s2=((a(2)+a(3)*c3)*P_wz-a(3)*s3*P_wx)/((P_wx)^(2)+(P_wz)^(2));
    c2=((a(2)+a(3)*c3)*P_wx+a(3)*s3*P_wz)/((P_wx)^(2)+(P_wz)^(2));
    
    theta2 = radtodeg(atan2(s2,c2));
    theta3 = radtodeg(atan2(s3,c3));

%% Control of the first angular value,if <0� or >180� choose the alternative solution given by redundancy
    if( theta2<0 || theta2>180)
	    s3=-sqrt(1-(c3)^(2));
		s2=((a(2)+a(3)*c3)*P_wz-a(3)*s3*P_wx)/((P_wx)^(2)+(P_wz)^(2));
        c2=((a(2)+a(3)*c3)*P_wx+a(3)*s3*P_wz)/((P_wx)^(2)+(P_wz)^(2));
        
        theta3 = radtodeg(atan2(s3,c3));
        theta2 = radtodeg(atan2(s2,c2));
        
        if abs(theta2)<0.001
            theta2=0;
        end

        theta4=orientazione_terna-theta2-theta3;
		theta=[theta1,theta2,theta3,theta4];
        % final offset fix
        theta(1) = -theta(1);
        theta(2) = 90-theta(2);
        return

    else %if found from inverse Kinematic is acceptable 
        theta4=orientazione_terna-theta2-theta3;
        theta=[theta1,theta2,theta3,theta4];
        % final offset fix
        theta(1) = -theta(1);
        theta(2) = 90-theta(2);
        return
    end

    theta=NaN; %no solution found
    if debug
        disp('98')
        keyboard
    end
    flag=1;
end