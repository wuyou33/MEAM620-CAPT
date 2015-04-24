clear all
close all

addpath('munkres')
N = 700;
M = 700;
R = 0.5;

max_v = 1;
t0 = 0;
max_allow_dist = 2*R*sqrt(2);

dist_scale = max_allow_dist*N;

full_points = zeros(2*N,2);
full_points(1,:) = rand(1,2)*dist_scale;

fprintf('Generating start and goal locations... ')
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
fprintf('Done! \n')

S = full_points(1:N,:);
G_pre = full_points(N+1:end,:);

tic
fprintf('Calculating distances... ')
D = zeros(N,M);
for i = 1:N
    D(i,:) = sum(bsxfun(@minus,G_pre,S(i,:)).^2,2)';
end
fprintf('Done! \n')

fprintf('Running the Hungarian algorithm... ')
[assignment, cost] = munkres(D);
lin_idx = sub2ind([N,M],1:N,assignment);
max_d = max(sqrt(D(lin_idx)));
fprintf('Done! \n')

tf = max_d/max_v;

G = G_pre(assignment,:);
toc

fprintf('Plotting... ')
plot_phase1(S,G,t0,tf,R)
fprintf('Done! \n')

