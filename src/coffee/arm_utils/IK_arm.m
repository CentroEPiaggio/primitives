function [theta,flag]=IK_arm(x,y,z,approach)
debug = 1;
if debug
    keyboard
end

a = [0,0.09,0.11,0.11];     %%Parameter A (Convention D-H)
flag=0;                     %if solution found return flag=0 otherwise flag=1
orientazione_terna=approach;% Orientation of Approach of the End-effector in the Plane X1-Z0

% compute the angle of the first joint
theta1=radtodeg(atan2(y,x));

%projection position of the penultimate joint of the serial chain in the X0-Y0 plane
P_wx=((sqrt(x^(2)+(y^(2))))-a(4)*cosd(orientazione_terna));
P_wz= z-(a(4)*sind(orientazione_terna));

%compute the cosine and sine of the second and third Joints
c3=(((P_wx)^(2))+((P_wz)^(2))-((a(2))^(2))-((a(3))^(2)))/(2*a(2)*a(3));
c3 = round(c3*1e8)/1e8; % dirty fix
if(abs(c3)>1)
    theta=[30,30,30,30];
    flag=1;
    return
end

s3=sqrt(1-(c3^(2)));
s2=((a(2)+a(3)*c3)*P_wz-a(3)*s3*P_wx)/((P_wx)^(2)+(P_wz)^(2));
c2=((a(2)+a(3)*c3)*P_wx+a(3)*s3*P_wz)/((P_wx)^(2)+(P_wz)^(2));


%% Control of the first angular value,if <0� or >180� choose the alternative solution given by redundancy
if( radtodeg(atan2(s2,c2))<10)||(radtodeg(atan2(s2,c2))>170)
    s3=-sqrt(1-(c3)^(2));
    s2=((a(2)+a(3)*c3)*P_wz-a(3)*s3*P_wx)/((P_wx)^(2)+(P_wz)^(2));
    c2=((a(2)+a(3)*c3)*P_wx+a(3)*s3*P_wz)/((P_wx)^(2)+(P_wz)^(2));
    if ( radtodeg(atan2(s2,c2))<10 || radtodeg(atan2(s2,c2))>170)
        theta=[30,30,30,30];               %not found solution
        flag=1;
        return
    end
    theta2=radtodeg(atan2(s2,c2));             %found angle of second Joint
    
    
    %control the angle of the third joint,checking the maximum and minimum limits
    %provided by piecewise linear functions "bound_up" and "bound_down"
    theta3=radtodeg(atan2(s3,c3));
    if (theta3<bound_up(theta2) && theta3>bound_down(theta2))
        theta3=radtodeg(atan2(s3,c3));     %found angle of third Joint
        theta4=orientazione_terna-theta2-theta3;
        theta=[theta1,theta2,theta3,theta4];
        return
    else  if(theta3>bound_up(theta2)|| theta3<bound_down(theta2))
            theta=[30,30,30,30];       %not found solution
            flag=1;
            return
        end
        
    end
end



%if found from inverse Kinematic is acceptable
if((radtodeg(atan2(s2,c2)))>10 && (radtodeg(atan2(s2,c2))<170))
    theta2=radtodeg(atan2(s2,c2));     %found angle of second Joint
    theta3=radtodeg(atan2(s3,c3));
    if (theta3<bound_up(theta2) && theta3>bound_down(theta2))
        theta3=radtodeg(atan2(s3,c3)); %found angle of second Joint
        theta4=orientazione_terna-theta2-theta3;
        theta=[theta1,theta2,theta3,theta4];
        return
    else  if(theta3>bound_up(theta2) || theta3<bound_down(theta2))
            theta=[30,30,30,30];   %not found solution
            flag=1;
            return
        end
        
    end
end
% 	theta=[30,30,30,30];
theta=NaN;
flag=1;





