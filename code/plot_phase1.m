function [] = plot_phase1(S,G,t0,tf,r)
% Plot and check results for simple 2D, min velocity^2 C-CAPT.
% ASSUMPTIONS:
%   Velocity constant (for velocity verification)

N = size(S,1);

% Time parameterization
beta = @(t) (-t0 + t)/(tf-t0);

% Robot patch points
theta = 0:0.1:2*pi;
robot_pts = r*[cos(theta'), sin(theta')];

% Initialize the plot
figure(1)
clf

plot(reshape([S(:,1)';G(:,1)';NaN(1,N)],[3*N,1]), ...
    reshape([S(:,2)';G(:,2)';NaN(1,N)],[3*N,1]),'+-k');

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

% Check velocities
max_v = max(sqrt(sum((G-S).^2,2))/(tf-t0));
disp(['Max velocity: ',num2str(max_v)]);

% Animate the motion
n_frames = 30;
dur = 1;
t = linspace(t0,tf,n_frames);
tic
for ii = 1:n_frames
    pos = (1-beta(t(ii)))*S + beta(t(ii))*G;
    for jj = 1:N
        dists = sqrt(sum(bsxfun(@minus,pos([1:(jj-1),(jj+1):N],:),pos(jj,:)).^2,2));
        collide = dists < 2*r;
        set(h_robots(jj),'xdata', pos(jj,1) + robot_pts(:,1), ...
            'ydata', pos(jj,2) + robot_pts(:,2));
        if any(collide)
            colliders = find(collide);
            colliders(colliders >= jj) = colliders(colliders >= jj) + 1;
            set(h_robots([colliders,jj]),'FaceColor',[1,0,0])
            disp(['COLLISION AT t=',num2str(t)]);
            return
        end
    end
    drawnow
    pause(dur/n_frames - toc);
    tic
end
