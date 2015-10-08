% funzione studenti Lucia
function [P] = vertex_matrix_n(n)
%this function returns a matrix with the vertices of a n-dimensional
%polyhedron

D = [0 : 2^n - 1];
B = dec2bin(D);
P = zeros(2^n, n);

for i=1:2^n
    for j=1:n
        temp=str2num(B(i,j));
        P(i,j)=temp;
    end
end
