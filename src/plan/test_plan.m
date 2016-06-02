%%

setenv('ROS_MASTER_URI','http://192.168.1.68:11311');
setenv('ROS_HOSTNAME','192.168.1.179');
setenv('ROS_IP','192.168.1.179');

rosinit

joint_pub = rospublisher('/iRobot_0/arm_joints',rostype.std_msgs_Int16MultiArray);
pause(0.1);
joint_msg = rosmessage(joint_pub);

cmd_msg = rosmessage('ff_fb_control/dd_control');
cmd_pub = rospublisher('/iRobot_0/control_topic',cmd_msg.MessageType);
pause(0.1);

mode_pub = rospublisher('/iRobot_0/mode',rostype.std_msgs_String);
pause(0.1);
mode_msg = rosmessage(mode_pub);
mode_msg.Data = 'safe';

special_pub = rospublisher('/iRobot_0/special',rostype.std_msgs_String);
pause(0.1);
special_msg = rosmessage(special_pub);
special_msg.Data = 'reset_odom';
%%

Ts=0.05;
%sec = 20;
sec = 10;
N=sec/Ts;

x=zeros(5,N);

% x(4,1:N/4)=0.1;
% x(4,N/4+1:N/4+9)=[0.1:-0.0125:0]';
% x(4,3*N/4-9:3*N/4-1)=[0:0.0125:0.1]';
% x(4,3*N/4:N)=0.1;

x(4,:)=0.1;

for i=1:N-1
    x(1,i+1) = x(1,i) + Ts*x(4,i)*cos(x(3,i)); % x
    x(2,i+1) = x(2,i) + Ts*x(4,i)*sin(x(3,i)); % y
    x(3,i+1) = x(3,i) + Ts*x(5,i); %theta
end

plot((1:N)*Ts,x)
grid on
legend('x','y','\theta','v','\omega')

joint_msg.Data = [-30 80 -80 40 60];


%%

send(mode_pub,mode_msg);
pause(0.1);
send(mode_pub,mode_msg);
pause(0.1);
send(special_pub,special_msg);
pause(0.1);

asd=1;

for j=1:N
    cmd_msg.DesiredX=x(1,j);
    cmd_msg.DesiredY=x(2,j);
    cmd_msg.DesiredTheta=x(3,j);
    cmd_msg.DesiredLinearVelocity=x(4,j);
    cmd_msg.DesiredAngularVelocity=x(5,j);
    send(cmd_pub, cmd_msg); %sending the message
    
%     if j>N/3 && asd==1
    if j>N/4 && asd==1
        send(joint_pub, joint_msg);
        asd=0;
    end
    
    pause(Ts); %ensure
end

%%

rosshutdown