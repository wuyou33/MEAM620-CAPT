function num_swaps = D_CAPT_function(S,G,t,h)

N = size(S,1);

robots = cell(N,1);

num_swaps = 0;

t_0 = t(1);
t_f = t(end);

X = S;
C = calculate_C(X,h);
for i_t = 1:N
    robots{i_t}.U = C(i_t,:);
    robots{i_t}.goal = G(i_t,:);
    robots{i_t}.pos = S(i_t,:);
    robots{i_t}.start = S(i_t,:);
    robots{i_t}.t_0 = t_0;

end

C_prev = zeros(N);

for i_t = 1:length(t)
    % get new time stamp
    t_c = t(i_t);

    % update the motion for all robots
    for i = 1:N
        robots{i}.pos = update_pos(robots{i},t_c,t_f);
        X(i,:) = robots{i}.pos;
    end

    % calculate the new C matrix
    C = calculate_C(X,h);

    % D-CAPT for each robot
    for i = 1:N
        robots{i}.U = (C(i,:) - C_prev(i,:)) > 0;

        while sum(robots{i}.U)

            j = find(robots{i}.U,1,'first');

            u = robots{j}.pos - robots{i}.pos;
            w = robots{j}.goal - robots{i}.goal;

            if dot(u,w) < 0

                num_swaps = num_swaps + 1;

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

end
