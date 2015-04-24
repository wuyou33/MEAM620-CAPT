clear all
close all

addpath('munkres')
N = 50;
M = 50;
R = 0.5;
max_v = 1;
t_0 = 0;

dist_scale = 2*sqrt(2)*R*N;


theta = 0:0.01:2*pi;
x = R*cos(theta);
y = R*sin(theta);

S = zeros(N,2);
for i = 1:N
    S(i,:) = rand(1,2)*dist_scale;
    if i > 1
        seed_flag = true;
        while seed_flag
            distances = sum(bsxfun(@minus,S(i,:),S(1:i-1,:)).^2,2);
            if min(distances) > 2*sqrt(2)*R
                seed_flag = false;
            else
                S(i,:) = rand(1,2)*dist_scale;
            end
        end
    end
end

G_pre = zeros(N,2);
for i = 1:M
    i
    G_pre(i,:) = rand(1,2)*dist_scale;
    if i > 1
        seed_flag = true;
        while seed_flag
            distances = [sum(bsxfun(@minus,G_pre(i,:),G_pre(1:i-1,:)).^2,2);...
                        sum(bsxfun(@minus,G_pre(i,:),S).^2,2)];
            if min(distances) > 2*sqrt(2)*R
                seed_flag = false;
            else
                G_pre(i,:) = rand(1,2)*dist_scale;
            end
        end
    end
end

figure(1000);
s_plot = plot(S(:,1),S(:,2),'r*');
hold on
g_plot = plot(G_pre(:,1),G_pre(:,2),'b*');
grid on
axis equal

D = zeros(N,M);

for i = 1:N
    D(i,:) = sum(bsxfun(@minus,G_pre,S(i,:)).^2,2)';
end

[assignment, cost] = munkres(D);
lin_idx = sub2ind([N,M],1:N,assignment);
max_d = max(sqrt(D(lin_idx)));

t_f = max_d/max_v;

G = G_pre(assignment,:);

plot_phase1(S,G,t_0,t_f,R)