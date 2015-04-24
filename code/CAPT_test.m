clear all
close all

addpath('munkres')
N = 5;
M = 5;
R = 0.5;

dist_scale = 5;
S = rand(N,2)*dist_scale;
G_pre = rand(M,2)*dist_scale;
max_v = 1;
t_0 = 0;

seed_flag = true;
while seed_flag
    D = zeros(N,M);
    for i = 1:N
        D(i,:) = sum(bsxfun(@minus,G_pre,S(i,:)).^2,2)';
    end
    if min(D(:)) > 2*sqrt(2)*R
        seed_flag = false;
    else
        S = rand(N,2)*dist_scale;
        G_pre = rand(M,2)*dist_scale;
    end
end

[assignment, cost] = munkres(D);
lin_idx = sub2ind([N,M],1:N,assignment);
max_d = max(sqrt(D(lin_idx)));

t_f = max_d/max_v;

G = G_pre(assignment,:);

