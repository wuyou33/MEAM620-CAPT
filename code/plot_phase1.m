function [] = plot_phase1(S,G,t0,tf,r)

N = size(S,1);

% Time parameterization
beta = @(t) (-t0 + t)/(tf-t0);

% Robot patch points
theta = 0:0.1:2*pi;
robot_pts = r*[cos(theta'), sin(theta')];

% Initialize the plot
figure(1)
clf

plot(reshape([S(:,1)';G(:,1)'],[2*N,1]), reshape([S(:,2)';G(:,2)'],[2*N,1]),'+-k');

hold on
h_robots = zeros(N,1);
for ii = 1:N
    h_robots(ii) = patch(S(ii,1) + robot_pts(:,1), S(ii,2) + robot_pts(:,2), ...
        [.5,.6,0.9]);
end
hold off

axis equal
xbound = [min([S(:,1); G(:,1)])-r, max([S(:,1); G(:,1)+r])];
ybound = [min([S(:,2); G(:,2)])-r, max([S(:,2); G(:,2)+r])];
set(gca,'xlim',xbound,'ylim',ybound)
title('C-CAPT Phase 1: 2D, Velocity Squared')

waitforbuttonpress

% Animate the motion
n_frames = 30;
dur = 1;
t = linspace(t0,tf,n_frames);
tic
for ii = 1:n_frames
    for jj = 1:N
        pos = (1-beta(t(ii)))*S(jj,:) + beta(t(ii))*G(jj,:);
        set(h_robots(jj),'xdata', pos(1) + robot_pts(:,1), ...
            'ydata', pos(2) + robot_pts(:,2));
    end
    drawnow
    pause(dur/n_frames - toc);
    tic
end
