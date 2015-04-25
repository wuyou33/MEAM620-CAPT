function traj = follower(C,n,t_vec,t)
valid = find(t >= t_vec);
idx = valid(end);
if idx >= length(t_vec)
    idx = idx -1;
end
coeff = C((idx-1)*2*n+1:idx*2*n,:);
traj = time_matrix(t,n)*coeff;
end