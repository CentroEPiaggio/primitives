rosshutdown
rosinit

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

rosshutdown