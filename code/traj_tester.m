%clear all
close all

X = [-1 0 0;...
    0 2 0;...
    1 0 0];
X_0 = [-1 -5 0;...
        0 0 0;...
        0 0 0];
X_f = [0 0 0;...
        0 0 0;...
        0 0 0];
ts = [0 5 6]';
n = 4;
C = min_n_traj(X,X_0,X_f,n,ts);

time = (0:0.001:ts(end))';
pos = zeros(length(time),3);
vel = zeros(length(time),3);
acc = zeros(length(time),3);
jerk = zeros(length(time),3);
c_idx = 1;
coeff = C(1:2*n,:);

for i = 1:length(time)
    t = time(i);
    traj = follower(C,n,ts,t);
    pos(i,:) = traj(1,:);
    vel(i,:) = traj(2,:);
    acc(i,:) = traj(3,:);
end

figure(1)
subplot(3,1,1)
plot(time,pos(:,1),'r-')
subplot(3,1,2)
plot(time,vel(:,1),'g-')
subplot(3,1,3)
plot(time,acc(:,1),'b-')

figure(2)
plot(pos(:,1),pos(:,2),'b-')
axis equal
grid on