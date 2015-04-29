function full_points = seed_random(N,r_base,dim)

M = N;

max_allow_dist = 2*r_base*sqrt(2);

dist_scale = [max_allow_dist*(N+M), max_allow_dist*(N+M)];

full_points = zeros(N+M,dim);
full_points(1,:) = rand(1,dim);

for i_t = 2:M+N
    seed_flag = true;
    pull_vect = full_points(1:i_t-1,:);

    while seed_flag
        new_point = rand(1,dim).*dist_scale;

        dists = sqrt(sum(bsxfun(@minus,pull_vect,new_point).^2,2));
        min_dist = min(dists);

        if min_dist > max_allow_dist
            full_points(i_t,:) = new_point;
            seed_flag = false;
        end
        
    end
    full_points(i_t,:) = new_point;

end
