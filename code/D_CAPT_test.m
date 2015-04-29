%clear all
%close all

dim = 2;
N = 25;
r_base = .5;
R = [r_base, r_base];

t_0 = 0;
t_f = 1;
dt = .001;
t = linspace(t_0,t_f,(t_f-t_0)/dt);

swap_test_flag = 1;

num_tests = 10;
hs = [1:10]*2*sqrt(2)*r_base;
swap_matrix = zeros(num_tests,numel(hs));

if swap_test_flag
    for H = 1:numel(hs)
        h = hs(H);

        for TRIAL_NUM = 1:num_tests
            fprintf('Trial %d of %d, h %d of %d... ',TRIAL_NUM,num_tests,H,numel(hs))

            full_points = seed_random(N,r_base,dim);
            S = full_points(1:N,:);
            S_save = S;
            G = full_points(N+1:end,:);

            num_swaps = D_CAPT_function(S,G,t,h);
            
            fprintf('Done! \n')
            swap_matrix(TRIAL_NUM,H) = num_swaps;
        end
    end
end
boxplot(swap_matrix)
save('dcapt_swap_data.mat','swap_matrix','num_tests','hs','N','r_base')
