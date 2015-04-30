function [S,G] = generate_line(r,N,dim)

d_s = 2*sqrt(2)*r;

G = [(1:N)'*d_s zeros(N,dim-1)];
S = -G;