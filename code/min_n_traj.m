function coeff = min_n_traj(X,X_0, X_f,n,t)
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