clear all
%close all

addpath('munkres')

dim = 3;

N = 10;
M = 4;
r_base = .5;

R = [r_base; r_base; r_base];

max_v = 1;
max_a = 1;

t0 = 0;
max_allow_dist = 2*r_base*sqrt(2);

dist_scale = [max_allow_dist*(N+M), max_allow_dist*(N+M), .25*max_allow_dist*(N+M)];

full_points = zeros(N+M,dim);
full_points(1,:) = rand(1,dim).*dist_scale;

fprintf('Generating start and goal locations... ')
for i = 2:M+N
    seed_flag = true;
    pull_vect = full_points(1:i-1,:);

    while seed_flag
        new_point = rand(1,dim).*dist_scale;

        dists = sqrt(sum(bsxfun(@minus,pull_vect,new_point).^2,2));
        min_dist = min(dists);

        if min_dist > max_allow_dist
            new_point = new_point .*[1,1,1];
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
fprintf('Done! \n')

assignment_save = assignment;
dm = ~assignment;
assignment(dm) = 1;

G = G_pre(assignment,:);
G(dm,:) = S(dm,:);

lin_idx = sub2ind([N,M],1:N,assignment);
max_d = max(sqrt(D(lin_idx)));

[~,ind] = min(abs(D(:) - max_d^2));
[ind_i, ind_j] = ind2sub([N,M],ind);
raw_dist = (G_pre(ind_j,:)-S(ind_i,:));
raw_dist = raw_dist.* [1,1,2.2];
max_d_a = sqrt(sum((raw_dist.^2)));

rescale = [1,1,4];

G = bsxfun(@times,G,rescale);
S = bsxfun(@times,S,rescale);
R = R'.*rescale;

tf = sqrt(max_d_a*5.773)/max_a + .1*(max_d_a)^(1/3);

toc

fprintf('Plotting... ')
plot_3D(S,G,t0,tf,R)
fprintf('Done! \n')

