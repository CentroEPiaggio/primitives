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

frame = list.get(1).asString();

x = list.get(2).asDouble();
y = list.get(3).asDouble();
z = list.get(4).asDouble();
qx = list.get(5).asDouble();
qy = list.get(6).asDouble();
qz = list.get(7).asDouble();
qw = list.get(8).asDouble();

sx = list.get(9).asDouble();
sy = list.get(10).asDouble();
sz = list.get(11).asDouble();

disp(['data: ', char(frame), ' ', num2str(x), ' ', num2str(y), ' ', num2str(z), ' ', num2str(qx), ' ', num2str(qy), ' ', num2str(qz), ' ', num2str(qw), ' ', num2str(sx), ' ', num2str(sy), ' ', num2str(sz),])

%%
read_port.close;