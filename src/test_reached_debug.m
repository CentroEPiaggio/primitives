distance = [];
state = [];
reachdiff = [];
for ii=1:T.nnodes
    [state(ii),reachdiff(ii)]=reached(T.Node{ii},z_goal,tol)
    distance(ii) = norm(T.Node{ii}(1:2) - z_goal(1:2));
end

figure
xxx = 1:T.nnodes;
plot(xxx,distance,'x',xxx,state,'k*',xxx,reachdiff,'ro')
grid minor
return
%%
fidx = 30

    reached(T.Node{ii},z_goal,tol)