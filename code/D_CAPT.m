dim = 2;    % dimensions of environment

N = 15;     % Number of robots
M = N;      % Number of goals
r_base = .5;% Robot radius

max_allow_dist = 2*r_base*sqrt(2); % Seeding separation

dist_scale = [max_allow_dist*(N+M), max_allow_dist*(N+M)];

full_points = zeros(N+M,dim);
full_points(1,:) = rand(1,dim);

fprintf('Generating start and goal locations... ')
billiards = true;
if ~billiards
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
    G = full_points(N+1:end,:);
else
    [S,G] = generate_billiards(r_base,N,dim);
    full_points = [S;G];
end

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

%% Plotting Initialization
% Patch object points
thetas = linspace(0,2*pi,20)';
r_c_x = r_base * cos(thetas);
r_c_y = r_base * sin(thetas);
h_c_x = h * cos(thetas);
h_c_y = h * sin(thetas);

% Axes boundaries
pad = min(h,5);

x_min = min(full_points(:,1))-pad;
x_max = max(full_points(:,1))+pad;

y_min = min(full_points(:,2))-pad;
y_max = max(full_points(:,2))+pad;

% Handles
figure(1)
clf
r_plots = zeros(N,1);
h_plots = zeros(N,1);
p_plots = zeros(N,1);
g_plots = zeros(N,1);
t_plots = zeros(N,1);

% Plotting
hold on
for i = 1:N
    h_plots(i) = patch(h_c_x + robots{i}.pos(1), h_c_y + robots{i}.pos(2), ...
        'b', 'facealpha', 0.2, 'edgealpha', 0);
    p_plots(i) = plot(0,0,'-','color',[1,0.5,0],'linewidth',2);
    g_plots(i) = plot(0,0,'--','color',[1,0.5,0],'linewidth',1);
end
for i = 1:N
    r_plots(i) = patch(r_c_x + robots{i}.pos(1), r_c_y + robots{i}.pos(2), ...
        'b','facecolor', [1,0.5,0], 'facealpha', 0.5, 'edgealpha', 1);
    t_plots(i) = text(robots{i}.pos(1), robots{i}.pos(2), num2str(i));
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
    cc_dist = zeros(N);
    for i = 1:N
        robots{i}.pos = update_pos(robots{i},t_c,t_f);
        X(i,:) = robots{i}.pos;
        paths{i}(i_t,:) = robots{i}.pos;
            
        % Collision checking
        for j = (i+1):N
            cc_dist(i,j) = sqrt(sum((robots{i}.pos - robots{j}.pos).^2));
            %fprintf([num2str(dist),'\n']);
            if cc_dist(i,j) < 2*r_base
                disp('COLLISION')
                set(r_plots(i), 'facecolor', [1,0,0], 'facealpha', 1, ...
                    'linewidth', 3, 'edgecolor', 'r');
                set(r_plots(j), 'facecolor', [1,0,0], 'facealpha', 1, ...
                    'linewidth', 3, 'edgecolor', 'r');
                return
            end
        end
    end
    
    % update plotting for all robots
    for i = 1:N
        set(h_plots(i), 'xdata', h_c_x + robots{i}.pos(1), 'ydata', h_c_y + robots{i}.pos(2));
        set(p_plots(i), 'xdata', paths{i}(1:i_t,1), 'ydata', paths{i}(1:i_t,2));
        set(g_plots(i), 'xdata', [robots{i}.pos(1), robots{i}.goal(1)], ...
            'ydata', [robots{i}.pos(2), robots{i}.goal(2)])
        set(r_plots(i), 'xdata', r_c_x + robots{i}.pos(1), 'ydata', r_c_y + robots{i}.pos(2));
        set(t_plots(i), 'position', robots{i}.pos);
    end
    
    % calculate the new C matrix
    C = calculate_C(X,h);
    U = C - C_prev;
    
    % D-CAPT for each robot
    while any(U(:))
        
        [i,j] = find(U,1,'first');

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

            U(i,:) = C(i,:);
            U(j,:) = C(j,:);
        end

        U(i,j) = false;
        U(j,i) = false;
        
    end
    
    % assign current C to previous C
    C_prev = C;
    
    if ~mod(i_t,5)
        drawnow
    end
    %pause(dt)
end

