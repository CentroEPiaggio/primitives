function idx_P_avail = CheckAvailablePrimitives(z_new,Ptree)
idx_P_avail = zeros(Ptree.nnodes,1);
z_check = fix_nans(z_new,Ptree.Node{1}.dimensions);  % conventionally Chi0 is the image space of the FIRST element of the primitive tree
for ii=2:Ptree.nnodes % conventionally Chi0 is the image space of the FIRST element of the primitive tree. Hence no need to check the first element
    prim = Ptree.Node{ii};
    if prim.check_extendable(z_check)
        idx_P_avail(ii)=1;
    end
end