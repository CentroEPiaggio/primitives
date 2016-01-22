function Times = getTimes(A,D,J,x,xhat,xbar)
debug = 0;
verbose = 0;
if debug,keyboard,end
%% definition of times
T1 = A/J;
T2 = x-2*T1;
T3 = T1;
T4 = xhat;
T5 = D/J;
T6 = xbar-2*T5;
T7 = T5;
Times = [T1 T2 T3 T4 T5 T6 T7];
if verbose
fprintf('%6s %6s %6s %6s %6s %6s %6s \n','T1','T2','T3','T4','T5','T6','T7');
fprintf('%6.2f %6.2f %6.2f %6.2f %6.2f %6.2f %6.2f \n',T1,T2,T3,T4,T5,T6,T7);
end
end