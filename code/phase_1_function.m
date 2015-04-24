function [ runtime ] = phase_1_function(N,M)

addpath('munkres')

R = 0.5;

max_v = 1;
t0 = 0;
max_allow_dist = 2*R*sqrt(2);

dist_scale = max_allow_dist*N;

full_points = zeros(2*N,2);
full_points(1,:) = rand(1,2)*dist_scale;

for i = 2:2*N
    seed_flag = true;
    pull_vect = full_points(1:i-1,:);

    while seed_flag
        new_point = rand(1,2)*dist_scale;

        dists = sqrt(sum(bsxfun(@minus,pull_vect,new_point).^2,2));
        min_dist = min(dists);

        if min_dist > max_allow_dist
            full_points(i,:) = new_point;
            seed_flag = false;
        end
        
    end
    full_points(i,:) = new_point;

end

S = full_points(1:N,:);
G_pre = full_points(N+1:end,:);

tic
D = zeros(N,M);
for i = 1:N
    D(i,:) = sum(bsxfun(@minus,G_pre,S(i,:)).^2,2)';
end

[assignment, cost] = munkres(D);
lin_idx = sub2ind([N,M],1:N,assignment);
max_d = max(sqrt(D(lin_idx)));

tf = max_d/max_v;

G = G_pre(assignment,:);
runtime = toc;


end

