% NOTE: use roslaunch to launch irobotcreate2 package

rosshutdown
rosinit

disp(' - publishing twist')

vel_pub = rospublisher('/iRobot_0/cmd_vel',rostype.geometry_msgs_Twist); %create a publisher
pause(1); %ensure publisher is setup

vel_msg = rosmessage(vel_pub); %msg from publisher, nice!

vel_msg.Linear.X=0;
vel_msg.Linear.Y=0;
vel_msg.Linear.Z=0;
vel_msg.Angular.X=0;
vel_msg.Angular.Y=0;
vel_msg.Angular.Z=0;

i=0:pi/8:2*pi;
des_vel = sin(i);

for j=1:numel(des_vel)
    vel_msg.Linear.X=des_vel(j);
    send(vel_pub, vel_msg); %sending the message
    pause(0.1); %ensure
end

pause(1); %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp(' - publishing joint angles')

joint_pub = rospublisher('/iRobot_0/arm_joints',rostype.std_msgs_Int16MultiArray);
joint_msg = rosmessage(joint_pub);

joint_msg.Data(5) = 0; %gripper

for j=1:4
    joint_msg.Data(j)=j*10;
end

send(joint_pub, joint_msg); %sending the message
pause(0.1); %ensure

rosshutdown