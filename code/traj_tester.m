%clear all
close all

X = [-1 0 0;...
    0 2 0;...
    1 1 0;...
    1 0 0];
X_0 = [0 0 0;...
        0 0 0;...
        0 0 0];
X_f = [0 0 0;...
        0 0 0;...
        0 0 0];
ts = [0 5 7 8]';
n = 4;
[c,A,B] = min_n_traj(X,X_0,X_f,n,ts);

time = (0:0.001:ts(end))';
pos = zeros(length(time),3);
vel = zeros(length(time),3);
acc = zeros(length(time),3);
jerk = zeros(length(time),3);
c_idx = 1;
coeff = c(1:2*n,:);

t_mat_3 = @(p)([1 p p^2 p^3;...
    0 1 2*p 3*p^2;...
    0 0 2 6*p]);
%{d
t_mat_7 = @(t)([1 t t^2 t^3 t^4 t^5 t^6 t^7;...
    0 1 2*t 3*t^2 4*t^3 5*t^4 6*t^5 7*t^6;...
    0 0 2 6*t 12*t^2 20*t^3 30*t^4 42*t^5]);
    %}
for i = 1:length(time)
    t = time(i);
    if t > ts(c_idx+1)
        c_idx = c_idx+1;
        coeff = c((c_idx-1)*2*n+1:c_idx*2*n,:);
    end
    traj = t_mat_7(t)*coeff;
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