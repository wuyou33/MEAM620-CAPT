clear all
close all

% set_points = [10, 15, 20, 30, 50, 70, 100, 150, 200, 300, 500, 700, 1000];
set_points = [1000];

runs_per = 25;

times = zeros(runs_per,numel(set_points));

for i = 1:numel(set_points)
    for j = 1:runs_per
        fprintf('N: %d, run: %d of %d, ',set_points(i),j,runs_per)
        times(j,i) = phase_1_function(set_points(i),set_points(i));
        fprintf('time: %.3f \n',times(j,i))
    end
end

x_test = linspace(0,max(set_points),1000);

boxplot(log(times))
set(gca,'yticklabel',num2str(exp(get(gca,'ytick')'),'%10.3e'))