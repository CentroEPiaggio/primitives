function discontinuo=checkdiscontinuity(T,E,Ptree)
discontinuo = false;
for jj=2:T.nnodes
    idx_sorgente = T.Parent(jj);
    idx_destinazione = jj;
    sorgente = T.get(idx_sorgente);
    destinazione = T.get(idx_destinazione);
    x=E{idx_sorgente,idx_destinazione}.x;
    
    prim = Ptree.Node{E{idx_sorgente,idx_destinazione}.primitive_ID};
    %%
    try
        test_condition_source = ~isequaln(round(x(prim.dimensions>0,1)*100)/100,round(sorgente(prim.dimensions>0)*100)/100);
    catch ME
        disp(ME.message);
        keyboard
    end
    if test_condition_source
        disp('discont sorgente')
        discontinuo = true;
    end
    try
        test_condition_destination = ~isequaln(round(x(prim.dimensions>0,end)*100)/100,round(destinazione(prim.dimensions>0)*100)/100);
    catch ME
        disp(ME.message);
        keyboard
    end
    if test_condition_destination
        disp('discont destinazione')
        discontinuo = true;
    end
    if discontinuo
        cprintf('*[0,0,0]*','Discontinuity in the edge from node %d to node %d\n',idx_sorgente,idx_destinazione);
        keyboard
    end
end
