%clear all
%close all

dim = 2;
N = 25;
r_base = .5;
R = [r_base, r_base];

robots = cell(N,1);
t_0 = 0;
t_f = 1;
dt = .001;
t = linspace(t_0,t_f,(t_f-t_0)/dt);

num_tests = 20;
hs = [1:10]*2*sqrt(2)*r_base;

swap_matrix = zeros(num_tests,numel(hs));

for H = 1:numel(hs)
    h = hs(H);
    
    for TRIAL_NUM = 1:num_tests
        fprintf('Trial %d of %d, h %d of %d... ',TRIAL_NUM,num_tests,H,numel(hs))

        full_points = seed_random(N,r_base,dim);
        S = full_points(1:N,:);
        S_save = S;
        G = full_points(N+1:end,:);
        
        num_swaps = 0;

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
        t_prev = t(1);


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
        fprintf('Done! \n')
        swap_matrix(TRIAL_NUM,H) = num_swaps;
    end
end

boxplot(swap_matrix)
save('dcapt_swap_data.mat','swap_matrix','num_tests','hs','N')
