clear all
%close all

dim = 2;

N = 5;
M = N;
r_base = .5;

R = [r_base, r_base];

max_v = 1;
max_a = 1;

t0 = 0;
max_allow_dist = 2*r_base*sqrt(2);

dist_scale = [max_allow_dist*(N+M), max_allow_dist*(N+M)];

full_points = zeros(N+M,dim);
full_points(1,:) = rand(1,dim);

fprintf('Generating start and goal locations... ')
for i_t = 2:M+N
    seed_flag = true;
    pull_vect = full_points(1:i_t-1,:);

    while seed_flag
        new_point = rand(1,dim).*dist_scale;

        dists = sqrt(sum(bsxfun(@minus,pull_vect,new_point).^2,2));
        min_dist = min(dists);

        if min_dist > max_allow_dist
            new_point = new_point;
            full_points(i_t,:) = new_point;
            seed_flag = false;
        end
        
    end
    full_points(i_t,:) = new_point;

end
fprintf('Done! \n')

S = full_points(1:N,:);
S_save = S;
G = full_points(N+1:end,:);

%% start D-CAPT
robots = cell(N,1);
t_0 = 0;
t_f = 1;
dt = .001;
t = linspace(t_0,t_f,(t_f-t_0)/dt);

h = 6*sqrt(2)*r_base;

X = S;
C = calculate_C(X,h);
for i_t = 1:N
    robots{i_t}.U = C(i_t,:);
    robots{i_t}.goal = G(i_t,:);
    robots{i_t}.pos = S(i_t,:);
    robots{i_t}.start = S(i_t,:);
    robots{i_t}.t_0 = t_0;
end

C_prev = C;
t_prev = t(1);

figure(1)
clf
r_plots = plot(X(:,1),X(:,2),'ro');
hold on
plot(G(:,1),G(:,2),'k*')
axis equal
grid on

for i_t = 2:length(t)
    % get new time stamp
    t_c = t(i_t);
    
    % update the motion for all robots
    for i = 1:N
        robots{i}.pos = update_pos(robots{i},t_c,t_f);
        X(i,:) = robots{i}.pos;
    end
    %set(r_plots,'xdata',X(:,1),'ydata',X(:,2))
    
    % calculate the new C matrix
    C = calculate_C(X,h);
    
    % D-CAPT for each robot
    for i = 1:N
        U_i = (C(i,:) - C_prev(i,:)) > 0;
    end
    
    % assign current C to previous C
    C_prev = C;
    
    %drawnow
    %pause(dt)
end