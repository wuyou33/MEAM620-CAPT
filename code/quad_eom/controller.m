function [F, M, trpy, drpy] = controller(qd, t, qn, params)
% CONTROLLER quadrotor controller
% The current states are:
% qd{qn}.pos, qd{qn}.vel, qd{qn}.euler = [roll;pitch;yaw], qd{qn}.omega
% The desired states are:
% qd{qn}.pos_des, qd{qn}.vel_des, qd{qn}.acc_des, qd{qn}.yaw_des, qd{qn}.yawdot_des
% Using these current and desired states, you have to compute the desired controls

% =================== Your code goes here ===================

persistent Pint Aint t_prev

% Initialization
if t < 0.01
    Pint = 0;
    Aint = 0;
    t_prev = 0;
end

dt = t - t_prev;
dt = min(eps,dt);


% Position Control ------------------------------------------
Pkp = [30;30;10];
Pkd = [12;12;5];
Pki = [0;0;0];%[0.001;0.001;0.001];

pos_err = qd{qn}.pos_des - qd{qn}.pos;
vel_err = qd{qn}.vel_des - qd{qn}.vel;
Pint = Pint + pos_err*dt;

a = qd{qn}.acc_des + [0; 0; 9.81] + ...
    Pkp.*pos_err + Pkd.*vel_err + Pki.*Pint;

% acceleration checks
a(3) = min(a(3),params.maxF/params.mass);
if a(3) < params.minF/params.mass
    max_angle = params.maxangle/(params.minF/params.mass - a(3));
    a(3) = params.minF;
else
    max_angle = params.maxangle;
end

a_xy = sqrt(sum(a(1:2).^2));
if norm(a) > params.maxF/params.mass
    max_xy = sqrt(sum((params.maxF/params.mass).^2 - a(3)^2));
    a(1:2) = a(1:2) * max_xy/a_xy;
end
a_xy = sqrt(sum(a(1:2).^2));
if a_xy > tan(max_angle)*a(3)
    max_xy = tan(max_angle)*a(3);
    a(1:2) = a(1:2) * max_xy/a_xy;
end

a_pitch = cos(-qd{qn}.euler(3))*a(1) - sin(-qd{qn}.euler(3))*a(2);
a_roll = sin(-qd{qn}.euler(3))*a(1) + cos(-qd{qn}.euler(3))*a(2);

% Desired roll, pitch and yaw
phi_des = atan2(-a_roll,a(3));
theta_des = atan2(a_pitch,a(3));
psi_des = qd{qn}.yaw_des;


% Attitude Control ------------------------------------------
Akp = [1000;1000;500];
Akd = [80;80;40];
Aki = [0;0;0];%[0.001;0.001;0.001];

att_tau = 30;

att_err = [phi_des; theta_des; psi_des] - qd{qn}.euler;
att_err(att_err > pi) = att_err(att_err > pi) - 2*pi;
att_err(att_err < -pi) = att_err(att_err < -pi) + 2*pi;
ome_err = [att_tau*att_err(1:2); qd{qn}.yawdot_des] - qd{qn}.omega;
Aint = Aint + att_err*dt;

tau_des = Akp.*att_err + Akd.*ome_err + Aki.*Aint;

% Thrust
F    = params.mass*a(3)/(cos(qd{qn}.euler(1))*cos(qd{qn}.euler(2)));
% Moment
M    = params.I*tau_des;

% Debugging 
%set(gca,'xlim',[-.5,.5] + qd{qn}.pos(1),'ylim',[-.5,.5] + qd{qn}.pos(2),'zlim',[-.5,.5] + qd{qn}.pos(3))
%{
global F_des T_des acc_des
F_des = [F_des, a_des];
T_des = [T_des, att_tau];
acc_des = [acc_des, qd{qn}.acc_des];
%}
%{
global att des_att
att = [att, qd{qn}.euler];
des_att = [des_att, [phi_des; theta_des; psi_des]];
%}

% =================== Your code ends here ===================

% Output trpy and drpy as in hardware
trpy = [F, phi_des, theta_des, psi_des];
drpy = [0, 0,       0,         0];

end
