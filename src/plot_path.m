% PLOT_PATH: plots the trajectory of the path passing through the nodes
% indicated in idx_path, e.g. idx_path = [1 2 4 5]
function plot_path(E,idx_path)
figure(14)
clf
hold on
t0 = 0;

for ii=1:length(idx_path)-1
    edge = E{idx_path(ii),idx_path(ii+1)};
    x = edge.x;
    time = edge.time + t0;
    t0 = time(end);
    plot(time,x)
    hold on
end

grid on

end