close all
clear all
clc

run ../src/utils/startup_lmp.m;
import primitive_library.*;

%%

cost_table = rand(10,3);

xmin = 0;
xmax = 10;
y_min = -1;
y_max = 1;
vmin = -0.1;
vmax = +0.1;
dimensioni = [1 1 0 0];
initial_extend = [0 0 NaN NaN];

Muovi = DD_move([xmin vmin; xmin vmax; xmax vmax; xmax vmin],[1 0],cost_table,'DD_Muovi',dimensioni,initial_extend);

%%

% x,y,theta,v
z_start=[0 0 0 0];
z_end=[1 1 0 0];

[feasible,cost,q,x,time] = Muovi.steering(z_start,z_end);

z_start=[0 0 0 0];
z_end=[0 1 pi 0];

[feasible,cost,q,x,time] = Muovi.steering(z_start,z_end);

z_start=[0 0 0 0];
z_end=[1 1 pi/2 0];

[feasible,cost,q,x,time] = Muovi.steering(z_start,z_end);

z_start=[0 0 0 0];
z_end=[1 0 pi+0.01 0];

[feasible,cost,q,x,time] = Muovi.steering(z_start,z_end);

%% test irobot dritto

z_start=[0 0 0 0];
z_end=[1 0 0 0];

[feasible,cost,q,x,time] = Muovi.steering(z_start,z_end);

%% test irobot curva

z_start=[0 0 0 0];
z_end=[1 1 1.57 0.1];

[feasible,cost,q,x,time] = Muovi.steering(z_start,z_end);

%% test irobot raccordo

z_start=[0 0 0 0];
z_end=[1 1 1.57 0.1];

[feasible,cost,q,x,time] = Muovi.steering(z_start,z_end);

kn = [x(1,1);x(2,1)];
x_total = x(1,:);
y_total = x(2,:);
th_total = x(3,:);
v_total = x(4,:);
w_total = x(5,:);

% z_start=z_end; NO! la end vera (da integrazione)
z_start = x(1:4,end);
z_end=z_start;
z_end(2)=z_end(2)+1;

[feasible,cost,q,x,time] = Muovi.steering(z_start,z_end);

kn = [kn, [x(1,1);x(2,1)]];
x_total = [x_total x(1,:)];
y_total = [y_total x(2,:)];
th_total = [th_total x(3,:)];
v_total = [v_total x(4,:)];
w_total = [w_total x(5,:)];

z_start = x(1:4,end);
z_end=z_start;
z_end(1)=z_end(1)-1;
z_end(2)=z_end(2)+1;
z_end(3)=z_end(3)+1.57;
z_end(4)=0;

[feasible,cost,q,x,time] = Muovi.steering(z_start,z_end);

kn = [kn, [x(1,1);x(2,1)], [x(1,end);x(2,end)]];
x_total = [x_total x(1,:)];
y_total = [y_total x(2,:)];
th_total = [th_total x(3,:)];
v_total = [v_total x(4,:)];
w_total = [w_total x(5,:)];

% raccordone

subplot(1,3,1)
hold on
plot(kn(1,:),kn(2,:),'sr')
plot(x_total,y_total)
axis equal
subplot(1,3,2)
plot(1:numel(v_total),v_total)
subplot(1,3,3)
plot(1:numel(w_total),w_total)

x = [x_total;y_total;th_total;v_total;w_total];


%%

setenv('ROS_MASTER_URI','http://192.168.1.68:11311');
setenv('ROS_HOSTNAME','192.168.1.179');
setenv('ROS_IP','192.168.1.179');

rosinit

Ts=0.1;

cmd_msg = rosmessage('ff_fb_control/dd_control');

cmd_pub = rospublisher('/iRobot_0/control_topic',cmd_msg.MessageType); %create a publisher
pause(1); %ensure publisher is setup

for j=1:numel(x(4,:))
    cmd_msg.DesiredX=x(1,j);
    cmd_msg.DesiredY=x(2,j);
    cmd_msg.DesiredTheta=x(3,j);
    cmd_msg.DesiredLinearVelocity=x(4,j);
    cmd_msg.DesiredAngularVelocity=x(5,j);
    send(cmd_pub, cmd_msg); %sending the message
    pause(Ts); %ensure
end

rosshutdown
