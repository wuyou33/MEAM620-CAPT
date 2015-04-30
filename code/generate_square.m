function [S,G] = generate_square(r,N,dim)

S = zeros(N,dim);
G = zeros(N,dim);

d_s = 2*sqrt(2)*r;

if N == 1
    num_side = 1;
elseif N <= 4
    num_side = 2;
else
    num_side = ceil(N/4+1);
end
idx = 1;
for i = 1:num_side
    if i > 1 && i < num_side
        G(idx,:) = [i 1]*d_s-repmat((num_side+1)*d_s/2,1,dim);
        idx = idx+1;
        if idx == N+1
            S = G*[-1 0; 0 -1]*2;
            return
        end
        G(idx,:) = [i num_side]*d_s-repmat((num_side+1)*d_s/2,1,dim);
        idx = idx+1;
        if idx == N+1
            S = G*[-1 0; 0 -1]*2;
            return
        end
    else
        for j = 1:num_side
            G(idx,:) = [i j]*d_s-repmat((num_side+1)*d_s/2,1,dim);
            idx = idx+1;
            if idx == N+1
                S = G*[-1 0; 0 -1]*2;
                return
            end
        end
    end
end