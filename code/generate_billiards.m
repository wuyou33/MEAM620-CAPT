function [S,G] = generate_billiards(r,N,dim)
if dim ~= 2
    S = [];
    G = [];
    return
end
S = zeros(N,dim);
G = zeros(N,dim);

d_s = 5*sqrt(2)*r;

num_cols = round(sqrt(N));
x_spacing = (1:num_cols)*d_s;
x_spacing = x_spacing-x_spacing(1)-(x_spacing(end)-x_spacing(1))/2;

num_rows = num_cols + ceil((N-1-num_cols^2)/num_cols);
y_spacing = (1:num_rows)*d_s;
y_spacing = y_spacing-y_spacing(1)-(y_spacing(end)-y_spacing(1))/2;

G(1,:) = [max(x_spacing) max(y_spacing)]*2;
S(1,:) = G(1,:)*[-1 0; 0 -1];
idx=2;
for i = 1:num_cols
    for j = 1:num_rows
        G(idx,:) = [x_spacing(i) y_spacing(j)];
        S(idx,:) = G(idx,:)*[-1 0; 0 -1]+2*sqrt(2)*r;
        idx = idx+1;
        if idx == N+1
            return
        end
    end
end