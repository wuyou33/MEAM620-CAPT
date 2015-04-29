function C = calculate_C(X,h)
N = size(X,1);
C = false(N);
for i = 1:N
    C(i,:) = sqrt(sum(bsxfun(@minus,X,X(i,:)).^2,2))'<h;
    C(i,i) = 0;
end