%run in terminal: yarpserver --write

clear all
clc

LoadYarp;

read_port=yarp.Port;
read_port.close;

read_port.open('/locomanipulation/command:i');

recv=yarp.Bottle;
recv.fromString('');

pause(1);
%%
recv.fromString('');
while(strcmp(recv.toString, ''))
    read_port.read(recv); % run in terminal: yarp write ... /matlab/read and write something
end

list = recv.get(0).asList();

command_obj = list.get(0).asString();

frame_obj = list.get(1).asString();

x_obj = list.get(2).asDouble();
y_obj = list.get(3).asDouble();
z_obj = list.get(4).asDouble();
qx_obj = list.get(5).asDouble();
qy_obj = list.get(6).asDouble();
qz_obj = list.get(7).asDouble();
qw_obj = list.get(8).asDouble();

sx_obj = list.get(9).asDouble();
sy_obj = list.get(10).asDouble();
sz_obj = list.get(11).asDouble();

disp([char(command_obj), ': ', char(frame_obj), ' ', num2str(x_obj), ' ', num2str(y_obj), ' ', num2str(z_obj), ' ', num2str(qx_obj), ' ', num2str(qy_obj), ' ', num2str(qz_obj), ' ', num2str(qw_obj), ' ', num2str(sx_obj), ' ', num2str(sy_obj), ' ', num2str(sz_obj),])

using_yarp=1;

%%
read_port.close;