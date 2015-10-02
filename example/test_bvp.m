%% solve bvp

xi = 0;
xf = 10;
vi = 0;
vf = 0;
q = [xi;xf;vi;vf];

t = linspace(0,1,100);
% solinit = bvpinit(t,[xf vf 0 0 0 0 1] );
% solinit = bvpinit(t,[rand rand rand 1 rand rand rand 1]);
% T=6;
solinit = bvpinit(t,[2;2;0;-1; 1]);
tic
sol = bvp4c(@dynamics_jerk,@dynamics_jerk_bc,solinit);
toc
close all
% T = mean(sol.y(5,:));
T=-mean(sol.y(end,:));
plot(sol.x*T,sol.y(1,:),sol.x*T,sol.y(2,:))
t = linspace(0,1,100);
u = deval(sol,t);
% plot(t,u)


% plot(sol.y(1,:),sol.y(2,:))