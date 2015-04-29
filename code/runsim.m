addpath(genpath('./'));

try
close 2 3 4 5
catch
end

%% Additional init script
%{
W = cat(3,[0,0,0; 1,0,0], ...
          [0,1,0; 1,1,0], ...
          [0,1,1; 1,1,1]);
t = [1, 1];
%}

start = num2cell(W(:,:,1),2);
stop = num2cell(W(:,:,end),2);

init_script;

nquad = length(start);

%% Run trajectory
trajectory = test_trajectory(start, stop, true); % without visualization

%{
figure(5)
accs = (trajectory{1}(2:end,4:6) - trajectory{1}(1:(end-1),4:6))/0.01;
subplot(4,1,1)
plot((1:size(accs,1))*0.01,accs(:,1))
subplot(4,1,2)
plot((1:size(accs,1))*0.01,accs(:,2))
subplot(4,1,3)
plot((1:size(accs,1))*0.01,accs(:,3))
subplot(4,1,4)
plot((1:size(accs,1))*0.01,sqrt(sum(accs.^2,2)))
%}