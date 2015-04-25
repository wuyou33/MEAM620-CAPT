function [] = plot_3D(S,G,t0,tf,r)
% Plot and check results for simple 2D, min velocity^2 C-CAPT.
% ASSUMPTIONS:
%   Velocity constant (for velocity verification)

% Initialization ==========================================================
N = size(S,1);

% Time parameterization
%coeff = quintic_generate([0,1],[t0,tf]);
%beta = @(t) quintic_follow(coeff, t);
n = 4;
coeff = min_n_traj([0 1]',[zeros(1,n-1)]',[zeros(1,n-1)]',n,[t0 tf]');
beta = @(t) follower(coeff,n,[t0 tf]',t);

% Robot patch points
[rx, ry, rz] = ellipsoid(0,0,0,r(1),r(2),r(3),20);

% Initialize the plot
figure(1)
clf

% Draw trajectories
plot3(reshape([S(:,1)';G(:,1)';NaN(1,N)],[3*N,1]), ...
    reshape([S(:,2)';G(:,2)';NaN(1,N)],[3*N,1]), ...
    reshape([S(:,3)';G(:,3)';NaN(1,N)],[3*N,1]),'+-k');

% Draw robots
hold on
h_robots = zeros(N,1);
for ii = 1:N
    h_robots(ii) = surf(S(ii,1) + rx, S(ii,2) + ry, S(ii,3) + rz, ...
        'facecolor', [.5,.6,0.9], 'facealpha', 0.5, 'edgealpha', 0);
end
hold off

% Prepare checks
% TODO: velocity checks
disps = G - S;

% Set axes properties
axis equal
xbound = [min([S(:,1); G(:,1)])-r(1), max([S(:,1); G(:,1)])+r(1)];
ybound = [min([S(:,2); G(:,2)])-r(2), max([S(:,2); G(:,2)])+r(2)];
zbound = [min([S(:,3); G(:,3)])-r(3), max([S(:,3); G(:,3)])+r(3)];
set(gca,'xlim',xbound,'ylim',ybound,'zlim',zbound)
title('C-CAPT Phase 1: 3D')

%waitforbuttonpress

% Animate the motion ======================================================
n_frames = 30;
dur = 1;
t = linspace(t0,tf,n_frames);
tic
for ii = 1:n_frames
    % Calculate new positions
    t_n = beta(t(ii));
    pos = (1-t_n(1))*S + t_n(1)*G;
    
    pos_scaled = bsxfun(@rdivide, pos, reshape(r,[1,3]));
    collided = false;
    
    % Update display and check collisions
    for jj = 1:N
        set(h_robots(jj),'xdata', pos(jj,1) + rx, ...
            'ydata', pos(jj,2) + ry, ...
            'zdata', pos(jj,3) + rz);
        
        dists = sqrt(sum(bsxfun(@minus,pos_scaled([1:(jj-1),(jj+1):N],:),pos_scaled(jj,:)).^2,2));
        collide = dists < 2;
        if any(collide)
            colliders = find(collide);
            colliders(colliders >= jj) = colliders(colliders >= jj) + 1;
            set(h_robots([colliders,jj]),'FaceColor',[1,0,0])
            collided = true;
        end
    end
    drawnow
    
    % Return if collided
    if collided
        disp(['COLLISION AT t=',num2str(t(ii))]);
        return
    end
    
    % Animation timing
   % pause(dur/n_frames - toc);
    tic
end

end

%{
function coeff = quintic_generate(pts, ts)
% Generate quintic spline trajectories with the option at each node to halt
% completely or fly through with continuous 4th derivative.

M = zeros(6);
C = zeros(6,1);

M(1:3,1:6) = [1 0 0 0 0 0;
              0 1 0 0 0 0;
              0 0 2 0 0 0];
C(1:3) = [pts(1);0;0];

M((end-2):end,(end-5):end) = [1, ts(end), ts(end)^2, ts(end)^3,   ts(end)^4,    ts(end)^5;
                              0, 1,      2*ts(end), 3*ts(end)^2, 4*ts(end)^3,  5*ts(end)^4;
                              0, 0,      2,         6*ts(end),  12*ts(end)^2, 20*ts(end)^3];
C((end-2):end) = [pts(end);0;0];

coeff = M \ C;

end

function ss = quintic_follow(coeff, t_n)
% Calculate the commanded position, velocity, and acceleration at a point
% in time given the quintic spline coefficients

t_vect = [1 0 0; 
          t_n 1 0; 
          t_n.^2 2*t_n 2; 
          t_n.^3 3*t_n.^2 6*t_n; 
          t_n.^4 4*t_n.^3 12*t_n.^2; 
          t_n.^5 5*t_n.^4 20*t_n.^3];
 ss = sum(bsxfun(@times,coeff(1:6),t_vect));
end
%}
