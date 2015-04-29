clear all
%close all

addpath('munkres')

dim = 3;

N = 5;
M = 10;
r_base = .15;

R = [r_base, r_base, r_base];

max_a = 2;

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
S_save = S;
G_pre = full_points(N+1:end,:);

tic

iters = ceil(M/N);

S_pass = repmat(S,[1,1,iters]);
G_pass = zeros(N,3,iters);
tf = zeros(1,iters);

for i = 1:iters

    fprintf('Calculating distances... ')
    D = zeros(N,M);
    for j = 1:N
        D(j,:) = sum(bsxfun(@minus,G_pre,S(j,:)).^2,2)';
    end
    fprintf('Done! \n')
    
    fprintf('Running the Hungarian algorithm %d of %d... ',i,iters)
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

    G_pass(:,:,i) = bsxfun(@times,G,rescale);
    S_pass(:,:,i) = bsxfun(@times,S,rescale);
    S = G;
    G_pre(assignment,:) = [];

    tf(i) = sqrt(max_d_a*5.773)/max_a + .1*(max_d_a)^(1/3);
    
    M = M-N;
    
end
toc

R = R.*rescale;

W_pass(:,:,1) = S_pass(:,:,1);
W_pass(:,:,2:iters+1) = G_pass;

fprintf('Plotting... ')
%plot_3D_multi_wp(W_pass,tf,R)

W = W_pass;
t = tf;
init_script;


trajectory = test_trajectory(num2cell(W(:,:,1),2), num2cell(W(:,:,end),2), true); % without visualization
fprintf('Done! \n')

