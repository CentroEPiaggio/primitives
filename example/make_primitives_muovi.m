function make_primitives_muovi(prim_data)
%% Extract data from struct
xi = prim_data.xi;
xf = prim_data.xf;
% yi = prim_data.yi;
% ypi = prim_data.ypi;
% ypf = prim_data.ypf;
Tend = prim_data.Tend;
Ts = prim_data.Ts;
xf_vec_len = prim_data.xf_vec_len;
% yf_vec_len = prim_data.yf_vec_len;
vx0_vec_len = prim_data.vx0_vec_len;
vxf_vec_len = prim_data.vxf_vec_len;
filepath = prim_data.filepath;
%% Generate Primitive .mat files
if ~exist(filepath,'dir')
    mkdir(filepath)
end

% Now using together x and y trajectories
xf_vec = linspace(0,xf,xf_vec_len);
% yf_vec = linspace(0,4,yf_vec_len);
vx0_vec = linspace(0,4,vx0_vec_len);
vxf_vec = linspace(0,4,vxf_vec_len);

disp(['Generating ' prim_data.name ' primitives...']);
% for y=1:length(yf_vec)
%     yf = yf_vec(y);
%     [~,traj_y_cart] = muovi(yi,yf,ypi,ypf,Tend,Ts);
%     [~,traj_y_cart] = muovi(0,1,0,0,Tend/2,Ts);
    for x=1:length(xf_vec)
        xf = xf_vec(x);
        for vi=1:length(vx0_vec)
            xpi = vx0_vec(vi);
            for vf=1:length(vxf_vec)
                xpf = vxf_vec(vf);
%                 [time,traj_x_cart] = muovi(xi,xf,xpi,xpf,Tend,Ts);
%                 [time,traj_x_cart] = trajectory(0,5,0,0,Tend/2,Ts);
[time,traj_x_cart] = trajectory(xi,xf,xpi,xpf,Tend,Ts);
                q_reference = [time(:)';traj_x_cart(:)'];
                savestr = strcat('save', [' ', filepath], 'primitiva_', prim_data.name,'_',num2str(x),...
                    '_',num2str(vi),'_',num2str(vf),'.mat q_reference');
                eval(savestr);
            end
        end
    end
% end
disp(['Generating ' prim_data.name ' primitives... DONE']);

