function [ desired_state ] = trajectory_generator(t, qn, W_in, t_in)
% TRAJECTORY_GENERATOR: Turn a Dijkstra or A* path into a trajectory
%
% NOTE: This function would be called with variable number of input
% arguments. In init_script, it will be called with arguments
% trajectory_generator([], [], map, path) and later, in test_trajectory,
% it will be called with only t and qn as arguments, so your code should
% be able to handle that. This can be done by checking the number of
% arguments to the function using the "nargin" variable, check the
% MATLAB documentation for more information.
%
% map: The map structure returned by your load_map function
% path: This is the path returned by your planner (dijkstra function)
%
% desired_state: Contains all the information that is passed to the
% controller, as in phase 2
%
% It is suggested to use "persistent" variables to store map and path
% during the initialization call of trajectory_generator, e.g.
% persistent map0 path0
% map0 = map;
% path0 = path;

% Quintic Interpolation along straight segments with line of sight checks
persistent coeff ts t_cum

% Parameters ====================================
yaw = 0;
yawdot = 0;

n = 3;  % Min snap

% Pre-compute ===================================
if isempty(t)
    [N,~,n_wp] = size(W_in);
    
    % Save waypoint segment durations
    ts = t_in;
    t_cum = cumsum([0,ts(1:(end-1))]);
    
    % Calculate and save spline coefficients
    for wp = 1:(n_wp-1)
        for robo = 1:N
            coeff{wp,robo} = min_n_traj([W_in(robo,:,wp);W_in(robo,:,wp+1)],zeros(n-1,3),zeros(n-1,3),...
                n,[0 ts(wp)]');
        end
    end
    
    return
end

% Run ===========================================
% Find which waypoint segment we are in
seg = find(t >= t_cum, 1, 'last');
t_s = t - t_cum(seg);

% Calculate desired position and derivatives from splines
pos = follower(coeff{seg,qn},n,[0 ts(seg)]',t_s);

% Output desired state
desired_state.pos = pos(1,:)';
desired_state.vel = pos(2,:)';
desired_state.acc = pos(3,:)';
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;

end

