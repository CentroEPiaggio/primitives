function discontinuo=checkdiscontinuity(T,E,Ptree)
discontinuo = false;
    for jj=2:T.nnodes
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
%         if round(x(prim.dimensions>0,1)*100)/100 ~= round(sorgente(prim.dimensions>0)*100)/100
        if ~isequaln(round(x(prim.dimensions==0,1)*100)/100,round(sorgente(prim.dimensions==0)*100)/100)
            disp('discont sorgente')
            discontinuo = true;
        end
%         if round(x(prim.dimensions>0,end)*100)/100 ~= round(destinazione(prim.dimensions>0)*100)/100
        if ~isequaln(round(x(:,end)*100)/100,round(destinazione(:)*100)/100)
            disp('discont destinazione')
            discontinuo = true;
        end
        if discontinuo
            keyboard
        end
    end
