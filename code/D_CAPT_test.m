%clear all
%close all

dim = 2;
N = 100;
r_base = .5;
R = [r_base, r_base];

t_0 = 0;
t_f = 1;
dt = .001;
t = linspace(t_0,t_f,(t_f-t_0)/dt);

plot_flag = 0;
swap_test_flag = 0;
optim_test_flag = 1;

num_tests = 10;

hs = [1:100]*2*sqrt(2)*r_base;
swap_matrix = zeros(num_tests,numel(hs));


if swap_test_flag
    fprintf('|SWAP TEST| \n')

    for H = 1:numel(hs)
        h = hs(H);

        for TRIAL_NUM = 1:num_tests
            fprintf('Trial %d of %d, h %d of %d... ',TRIAL_NUM,num_tests,H,numel(hs))

            full_points = seed_random(N,r_base,dim);
            [num_swaps,paths] = D_CAPT_function(full_points,t,h,N,r_base,plot_flag);
            
            fprintf('Done! \n')
            swap_matrix(TRIAL_NUM,H) = num_swaps;
        end
    end
    %boxplot(swap_matrix)
    save('dcapt_swap_data.mat','swap_matrix','num_tests','hs','N','r_base')
end

Ns = [1:5:11];
h = 2*2*r_base*sqrt(2);
dcapt_matrix = zeros(num_tests,numel(Ns));
ccapt_matrix = zeros(num_tests,numel(Ns));

if optim_test_flag
    fprintf('|OPTIM TEST| \n')

    for num_bots = 1:numel(Ns)
        N = Ns(num_bots);

        for TRIAL_NUM = 1:num_tests
            fprintf('Trial %d of %d, N %d of %d... ',TRIAL_NUM,num_tests,num_bots,numel(Ns))

            full_points = seed_random(N,r_base,dim);
            [num_swaps,paths] = D_CAPT_function(full_points,t,h,N,r_base,plot_flag);

            total_dcapt_dist = 0;

            for i = 1:numel(paths)
                total_dcapt_dist = total_dcapt_dist+sum(sqrt(sum(diff(paths{1}).^2,2)));
            end
            dcapt_matrix(TRIAL_NUM,num_bots) = total_dcapt_dist;

            c_capt_dist = C_CAPT_function(full_points,N);

            ccapt_matrix(TRIAL_NUM,num_bots) = c_capt_dist;

            fprintf('Done! \n')
        end
    end
    save('dcapt_ccapt_data.mat','dcapt_matrix','ccapt_matrix','Ns','num_tests')

end