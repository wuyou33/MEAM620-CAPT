function [coeff,A,B] = min_n_traj(X,X_0, X_f,n,t)
[N,M] = size(X);

if size(X_0,1) ~= n-1 || size(X_f,1) ~= n-1 || size(X,1) ~= length(t)
    return
end

B = zeros(2*(N-1)*n,M);

% equalities
B(1:2*n,:) = [X(1,:);...
    X_0;...
    X(end,:);...
    X_f];
for i = 1:N-2
    B(2*n + 2*n*(i-1)+1:2*n+2*n*i,:) = [X(i+1,:);...
            zeros(2*(n-1),M);...
            X(i+1,:)];
end

A = zeros(2*(N-1)*n);
% end point constraints
for i = 1:n
    den = 1-i:2*n-i;
    d = factorial(0:2*n-1)./factorial(den.*(den>0));
    mask = [zeros(1,i-1) ones(1,2*n-i+1)];
    assignment_1 = d.*t(1).^((1-i:2*n-i).*((1-i:2*n-i)>0));
    assignment_1(~mask) = 0;
    assignment_2 = d.*t(end).^((1-i:2*n-i).*((1-i:2*n-i)>0));
    assignment_2(~mask) = 0;
    
    A(i,1:2*n) = assignment_1;
    A(n+i,end-2*n+1:end) = assignment_2;
end

idx_offset = 2*n;
%{d
% in between constraints
for j = 1:N-2
    A(idx_offset+((j-1)*2*n+1),(j-1)*2*n+1:j*2*n) = t(j+1).^(0:2*n-1);
    for i = 2:2*n-1
        den = 1-i:2*n-i;
        d = [factorial(0:2*n-1)./factorial(den.*(den>0)) -factorial(0:2*n-1)./factorial(den.*(den>0))];
        mask = repmat([zeros(1,i-1) ones(1,2*n-i+1)],1,2);
        assignment_1 = d.*repmat(t(j+1).^((1-i:2*n-i).*((1-i:2*n-i)>0)),1,2);
        assignment_1(~mask) = 0;
        
        A(idx_offset+((j-1)*2*n+i),(j-1)*2*n+1:(j-1)*2*n+4*n) = assignment_1;
    end
    A(idx_offset+((j-1)*(2*n)+2*n),j*2*n+1:(j+1)*2*n) = t(j+1).^(0:2*n-1);
end

coeff = A\B;
%{
A = [1 t_0 t_0^2 t_0^3 0 0 0 0;...
    0 1 2*t_0 3*t_0^2 0 0 0 0;...
    0 0 0 0 1 t_2 t_2^2 t_2^3;...
    0 0 0 0 0 1 2*t_2 3*t_2^2;...
    1 t_1 t_1^2 t_1^3 0 0 0 0;...
    0 0 0 0 1 t_1 t_1^2 t_1^3;...
    0 1 2*t_1 3*t_1^2 0 -1 -2*t_1 -3*t_1^2;...
    0 0 2 6*t_1 0 0 -2 -6*t_1];
%}
%}
%{
coeff = [1 0 0 0 0 0 0 0;...
         0 1 0 0 0 0 0 0;...
         0 0 2 0 0 0 0 0;...
         0 0 0 6 0 0 0 0;...
         1 t t^2 t^3 t^4 t^5 t^6 t^7;...
         0 1 2*t 3*t^2 4*t^3 5*t^4 6*t^5 7*t^6;...
         0 0 2 6*t 12*t^2 20*t^3 30*t^4 42*t^5;...
         0 0 0 6 24*t 60*t^2 120*t^3 210*t^4]\X;
%}
%end