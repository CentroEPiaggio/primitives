% how many nodes n we can have so that the definition that Frazzoli et. al
% give is computable (given it is based on the Euler Gamma function that
% appears in the volume of the n-ball sphere)???
% Maybe there is another way to compute it besides
% https://en.wikipedia.org/wiki/Volume_of_an_n-ball. Investigate.
clc
gam = 0.1;
n = 1:1:1000;
for ii=1:length(n)
    volume = gam*log(n(ii))/n(ii);
    radius_elevated_n = gamma(1+n(ii)/2)/pi^(n(ii)/2)*volume;
    radius(ii) = radius_elevated_n^(1/n(ii));
end
close all
plot(n,radius)
grid on
xlabel('number of nodes in the graph')

hold on
radius = radius(~isinf(radius));
maxrad = max(radius);
plot([n(1) n(end)],[maxrad maxrad],'r');
legend('radius of the n-ball',['maximum: ' num2str(maxrad)])
title('Above the maximum value the radius formula returns Inf values')