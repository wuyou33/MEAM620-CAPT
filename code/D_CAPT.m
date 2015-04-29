dim = 2;    % dimensions of environment

N = 10;     % Number of robots
M = N;      % Number of goals
r_base = .5;% Robot radius

max_allow_dist = 2*r_base*sqrt(2); % Seeding separation

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
paths = cell(N,1);
t_0 = 0;
t_f = 1;
dt = .001;
t = linspace(t_0,t_f,(t_f-t_0)/dt);

h = 4*sqrt(2)*r_base;

X = S;
C = calculate_C(X,h);
for i_t = 1:N
    robots{i_t}.U = C(i_t,:);
    robots{i_t}.goal = G(i_t,:);
    robots{i_t}.pos = S(i_t,:);
    robots{i_t}.start = S(i_t,:);
    robots{i_t}.t_0 = t_0;
    paths{i_t} = zeros(length(t),2);
end

C_prev = zeros(N);
t_prev = t(1);

thetas = linspace(0,2*pi,20)';
r_c_x = r_base * cos(thetas);
r_c_y = r_base * sin(thetas);
h_c_x = h * cos(thetas);
h_c_y = h * sin(thetas);

pad = min(h,5);

x_min = min(full_points(:,1))-pad;
x_max = max(full_points(:,1))+pad;

y_min = min(full_points(:,2))-pad;
y_max = max(full_points(:,2))+pad;

figure(1)
clf
r_plots = zeros(N,1);
h_plots = zeros(N,1);
p_plots = zeros(N,1);
g_plots = zeros(N,1);

hold on
for i = 1:N
    h_plots(i) = patch(r_c_x + robots{i}.pos(1), r_c_y + robots{i}.pos(2), ...
        'b', 'facealpha', 0.2, 'edgealpha', 0);
    p_plots(i) = plot(0,0,'-','color',[1,0.5,0],'linewidth',2);
    g_plots(i) = plot(0,0,'--','color',[1,0.5,0],'linewidth',2);
end
for i = 1:N
    r_plots(i) = patch(h_c_x + robots{i}.pos(1), h_c_y + robots{i}.pos(2), ...
        'b','facecolor', [1,0.5,0], 'facealpha', 0.5, 'edgealpha', 1);
end

plot(G(:,1),G(:,2),'k+','markersize',8)
plot(S(:,1),S(:,2),'kx','markersize',8)

axis equal
grid on
set(gca,'color',1*ones(1,3))
xlim([x_min,x_max])
ylim([y_min,y_max])

for i_t = 1:length(t)
    % get new time stamp
    t_c = t(i_t);
    
    % update the motion for all robots
    for i = 1:N
        robots{i}.pos = update_pos(robots{i},t_c,t_f);
        X(i,:) = robots{i}.pos;
    end
    
    % update plotting for all robots
    for i = 1:N
        set(r_plots(i), 'xdata', r_c_x + robots{i}.pos(1), 'ydata', r_c_y + robots{i}.pos(2));
        set(h_plots(i), 'xdata', h_c_x + robots{i}.pos(1), 'ydata', h_c_y + robots{i}.pos(2));
        
        paths{i}(i_t,:) = robots{i}.pos;
        set(p_plots(i), 'xdata', paths{i}(1:i_t,1), 'ydata', paths{i}(1:i_t,2));
        set(g_plots(i), 'xdata', [robots{i}.pos(1), robots{i}.goal(1)], ...
            'ydata', [robots{i}.pos(2), robots{i}.goal(2)])
    end
    
    % calculate the new C matrix
    C = calculate_C(X,h);
    
    % D-CAPT for each robot
    for i = 1:N
        robots{i}.U = (C(i,:) - C_prev(i,:)) > 0;
        %robots{i}.U = robots{i}.U & C(i,:);
        
        while sum(robots{i}.U)
            
            j = find(robots{i}.U,1,'first');
            
            u = robots{j}.pos - robots{i}.pos;
            w = robots{j}.goal - robots{i}.goal;
            
            if dot(u,w) < 0
                g_i = robots{i}.goal;
                g_j = robots{j}.goal;
                
                robots{i}.goal = g_j;
                robots{i}.t_0 = t_c;
                robots{i}.start = robots{i}.pos;

                robots{j}.goal = g_i;
                robots{j}.t_0 = t_c;
                robots{j}.start = robots{j}.pos;
                
                robots{i}.U = C(i,:);
                robots{j}.U = C(j,:);
                
            end
            
            robots{i}.U(j) = false;
            robots{j}.U(i) = false;
            
        end
        
    end
    
    % assign current C to previous C
    C_prev = C;
    
    if ~mod(i_t,5)
        drawnow
    end
    %pause(dt)
end

