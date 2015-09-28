%% solve ivp

xi = 0;
xf = 10;
vi = 0;
vf = 0;
q = [xi;xf;vi;vf];

t = linspace(0,1);
% solinit = bvpinit(t,[xf vf 0 0 0 0 1] );
solinit = bvpinit(t,[rand rand rand rand rand rand 1]);

sol = bvp4c(@dynamics_jerk,@dynamics_jerk_bc,solinit);