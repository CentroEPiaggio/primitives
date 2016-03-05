load test_a_2016_03_05_16_26_03.mat;


%%
opt_plan=extract_plan(T,E,opt_path)
path_length = opt_plan.nnodes;

t = [];
x = [];
time_offset = 0;
figure
hold on
for ii=2:path_length
    t_ii = time_offset + opt_plan.Node{ii}.time;
    x_ii = opt_plan.Node{ii}.x;
    t = [t, t_ii];
    time_offset = t(end);
    x = [x x_ii];
    plot(t_ii,x_ii);
end
grid on
plot(t(1)*[1 1],z_init(1:2),'ro')
plot(t(end)*[1 1],z_goal(1:2),'ro')

w = gradient(x(3,:),t);
plot(t,w)

figure
subplot(1,2,1)
plot(x(1,:),x(2,:),'k','linewidth',2)
subplot(1,2,2)
x(5,isnan(x(5,:))>0)=0
plot3(x(1,:),x(2,:),x(5,:),'k-d','linewidth',4)

bias=[];
bias_cell={};
asd=1;
for ii=1:10:size(x(1,:),2)
    bias = x(:,ii);
    bias(4,end)=0.1;
    bias(5,end)=0;
    bias_cell{asd} = bias;
    asd = asd+1;
    bias(5,end)=1;
    bias_cell{asd} = bias;
    asd = asd+1;
end
% remind: x is [x,y,theta,v,w]

xlabel('time [s]');

x = [x ; w];

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
%%

joint_msg.Data = [-30 80 -80 40 60];


%%
Ts=mean(diff(t));
asd=0;

for j=1:numel(x(4,:))
    cmd_msg.DesiredX=x(1,j);
    cmd_msg.DesiredY=x(2,j);
    cmd_msg.DesiredTheta=x(3,j);
    cmd_msg.DesiredLinearVelocity=x(4,j);
    cmd_msg.DesiredAngularVelocity=x(5,j);
    send(cmd_pub, cmd_msg); %sending the message
    
    if j>numel(x(4,:))/4 && asd==1
        send(joint_pub, joint_msg);
        asd=0;
    end
    
    pause(Ts); %ensure
end

%%

rosshutdown