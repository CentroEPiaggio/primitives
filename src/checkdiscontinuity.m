function discontinuo=checkdiscontinuity(T,E,Ptree)
discontinuo = false;
    for jj=2:T.nnodes
%         keyboard
        idx_sorgente = T.Parent(jj);
        idx_destinazione = jj;
        sorgente = T.get(idx_sorgente);
        destinazione = T.get(idx_destinazione);
        x=E{idx_sorgente,idx_destinazione}.x;
        %% check with primitive type DIRTY FIX
        if strcmp(E{idx_sorgente,idx_destinazione}.primitive,'Muovi')
            prim = Ptree.Node{1};
        elseif strcmp(E{idx_sorgente,idx_destinazione}.primitive,'Eleva')
            prim = Ptree.Node{2};
        else
            keyboard
        end
        %%
        if x(prim.dimensions>0,1) ~= sorgente(prim.dimensions>0)
            disp('discont sorgente')
            discontinuo = true;
        end
        if x(prim.dimensions>0,end) ~= destinazione(prim.dimensions>0)
            disp('discont destinazione')
            discontinuo = true;
        end
    end
