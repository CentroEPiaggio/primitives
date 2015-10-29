function discontinuo=checkdiscontinuity(T,E)
discontinuo = false;
    for jj=2:T.nnodes
        idx_sorgente = T.Parent(jj);
        idx_destinazione = jj;
        sorgente = T.get(idx_sorgente);
        destinazione = T.get(idx_destinazione);
        x=E{idx_sorgente,idx_destinazione}.x;
        if x(1:2,1) ~= sorgente(1:2)
            disp('cazzo')
            discontinuo = true;
        end
        if x(1:2,end) ~= destinazione(1:2)
            disp('cazzo')
            discontinuo = true;
        end
    end
