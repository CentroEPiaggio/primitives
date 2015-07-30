P = tree; % initialize tree
idx_primitive_next = 0;

cost_table = rand(10,3);
Forward = PrimitiveFun([-1 -1; -1 1; 1 -1; 1 1]*0.5,[1 0],cost_table,'Forward');
P = P.addnode(idx_primitive_next,Forward)

idx_primitive_next = idx_primitive_next+1;

Left = PrimitiveFun([-1 -1; -1 1; 1 -1; 1 1]*0.4,[10 0],cost_table,'Left');
P = P.addnode(idx_primitive_next,Left)

idx_primitive_next = idx_primitive_next+1;

Right = PrimitiveFun([-1 -1; -1 1; 1 -1; 1 1]*0.3,[0.1 0],cost_table,'Right');
P = P.addnode(idx_primitive_next,Right)

idx_primitive_next = idx_primitive_next+1;