function T = time_matrix(t,n)

T = zeros(3,2*n);

for i = 1:3
    den = 1-i:2*n-i;
    d = factorial(0:2*n-1)./factorial(den.*(den>0));
    mask = [zeros(1,i-1) ones(1,2*n-i+1)];
    assignment_1 = d.*t.^((1-i:2*n-i).*((1-i:2*n-i)>0));
    assignment_1(~mask) = 0;
    
    T(i,1:2*n) = assignment_1;
end