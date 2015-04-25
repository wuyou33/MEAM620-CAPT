function [] = plot_3D_multi_wp(W,t,r)
% Plot and check results for simple 2D, min velocity^2 C-CAPT.
% ASSUMPTIONS:
%   Velocity constant (for velocity verification)

% Initialization ==========================================================
[N,~,n_wp] = size(W);

% Time parameterization
t0 = [0, cumsum(t(1:(end-1)))];
n = 4;
coeff = cell(n_wp-1,1);
beta = cell(n_wp-1,1);
for ii = 1:(n_wp-1)
    coeff{ii} = min_n_traj([0 1]',zeros(n-1,1),zeros(n-1,1),n,[0 t(ii)]');
    beta{ii} = @(tn) follower(coeff{ii},n,[0 t(ii)]',tn);
end

% Robot patch points
[rx, ry, rz] = ellipsoid(0,0,0,r(1),r(2),r(3),20);

% Initialize the plot
figure(1)
clf

% Draw trajectories
if N > 1
    plot3(reshape([squeeze(W(:,1,:))';NaN(1,N)],[N*(n_wp+1),1]), ...
        reshape([squeeze(W(:,2,:))';NaN(1,N)],[N*(n_wp+1),1]), ...
        reshape([squeeze(W(:,3,:))';NaN(1,N)],[N*(n_wp+1),1]),'+-k');
else
    plot3(reshape([squeeze(W(:,1,:));NaN(1,N)],[N*(n_wp+1),1]), ...
        reshape([squeeze(W(:,2,:));NaN(1,N)],[N*(n_wp+1),1]), ...
        reshape([squeeze(W(:,3,:));NaN(1,N)],[N*(n_wp+1),1]),'+-k');
end

% Draw robots
hold on
h_robots = zeros(N,1);
for n = 1:N
    h_robots(n) = surf(W(n,1,1) + rx, W(n,2,1) + ry, W(n,3,1) + rz, ...
        'facecolor', [.5,.6,0.9], 'facealpha', 0.5, 'edgealpha', 0);
end
hold off

% Prepare checks
% TODO: velocity checks

% Set axes properties
axis equal
xbound = [min(min(W(:,1,:)))-r(1), max(max(W(:,1,:)))+r(1)];
ybound = [min(min(W(:,2,:)))-r(2), max(max(W(:,2,:)))+r(2)];
zbound = [min(min(W(:,3,:)))-r(3), max(max(W(:,3,:)))+r(3)];
set(gca,'xlim',xbound,'ylim',ybound,'zlim',zbound)
title('C-CAPT Phase 1: 3D')

%waitforbuttonpress

% Animate the motion ======================================================
frame_rate = 1/30;
ts = cell(n_wp-1,1);
n_frames = zeros(n_wp-1,1);
for ii = 1:(n_wp-1)
    ts{ii} = 0:frame_rate:t(ii);
    n_frames(ii) = length(ts{ii});
end
for w = 1:(n_wp-1)
    for k = 1:n_frames(w)
        % Calculate new positions
        t_n = beta{w}(ts{w}(k));
        pos = (1-t_n(1))*W(:,:,w) + t_n(1)*W(:,:,w+1);

        pos_scaled = bsxfun(@rdivide, pos, reshape(r,[1,3]));
        collided = false;

        % Update display and check collisions
        for n = 1:N
            set(h_robots(n),'xdata', pos(n,1) + rx, ...
                'ydata', pos(n,2) + ry, ...
                'zdata', pos(n,3) + rz);

            dists = sqrt(sum(bsxfun(@minus,pos_scaled([1:(n-1),(n+1):N],:),pos_scaled(n,:)).^2,2));
            collide = dists < 2;
            if any(collide)
                colliders = find(collide);
                colliders(colliders >= n) = colliders(colliders >= n) + 1;
                set(h_robots([colliders,n]),'FaceColor',[1,0,0])
                collided = true;
            end
        end
        drawnow

        % Return if collided
        if collided
            disp(['COLLISION AT t=',num2str(t0{w}+ts{w}(k))]);
            return
        end

        % Animation timing
    end
end

end