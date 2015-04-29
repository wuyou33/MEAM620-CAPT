function [total_cost] = C_CAPT_function(full_points,N)

addpath('./munkres/')

S = full_points(1:N,:);
G_pre = full_points(N+1:end,:);

M = N;

D = zeros(N,M);
for i = 1:N
    D(i,:) = sum(bsxfun(@minus,G_pre,S(i,:)).^2,2)';
end

[assignment, cost] = munkres(D);
G = G_pre(assignment,:);

total_cost = sum(sqrt(sum((S-G).^2,2)),1);

end