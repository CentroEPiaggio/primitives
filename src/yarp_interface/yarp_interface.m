%run in terminal: yarpserver --write

clear all
clc

LoadYarp;

read_port=yarp.Port;
write_port=yarp.Port;
%first close the port just in case
read_port.close;
write_port.close;

read_port.open('/matlab/read');
write_port.open('/matlab/write');

sent=yarp.Bottle;
sent.fromString('ciao');

recv=yarp.Bottle;
recv.fromString('');

pause(1);
%%

write_port.write(sent); % run in terminal: yarp read ... /matlab/write

%%
while(strcmp(recv.toString, ''))
    read_port.read(recv); % run in terminal: yarp write ... /matlab/read and write something
end
disp(recv);

%%
read_port.close;
write_port.close;