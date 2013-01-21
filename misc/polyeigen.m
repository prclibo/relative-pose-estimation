clc
clear all

d = 5; 
n = 56; 
m = 21; 
a = rand(1, n); 
b = rand(1, n); 

deg = zeros(3, 0); 
for i = 0:5
    for j = 0:5
        for k = 0:5
            if (i + j + k <= 5)
                deg(:, end + 1) = [i; j; k]; 
            end
        end
    end
end

deg2idx = @(x) find(deg(1, :) == x(1) & deg(2, :) == x(2) & deg(3, :) == x(3)); 

M = zeros(2, 56); 
M(1, :) = a; 
M(2, :) = b; 
for i = 0:3
    for j = 0:3
        for k = 0:3
            if (i + j + k <= 3)
                M(end + 1, :) = 0; 
                M(end, deg2idx([i + 2, j, k])) = 1; 
                M(end, deg2idx([i, j + 2, k])) = 1; 
                M(end, deg2idx([i, j, k + 2])) = 1; 
                M(end, deg2idx([i, j, k])) = -1; 
            end
        end
    end
end

M = M(1:m, :); 

v_deg = deg(1:2, deg(3, :) == 0); 
C = cell(6, 1); 
for ci = 1:6
    C{ci} = zeros(m); 
    z_deg = ci - 1; 
    zv_deg = [v_deg; z_deg * ones(1, m)]; 
    for i = 1:m
        idx = deg2idx(zv_deg(:, i)); 
        if ~isempty(idx)
            C{ci}(:, i) = M(:, idx); 
        end        
    end
end

[U, D] = polyeig(C{1}, C{2}, C{3}, C{4}, C{5}, C{6}); 

v_deg2idx = @(x) find(v_deg(1, :) == x(1) & v_deg(2, :) == x(2)); 

for i = 1:numel(D)
    if ~isinf(D(i)) & ~isnan(D(i)) & isreal(D(i))
        z = D(i); 
        x = U(v_deg2idx([1, 0]), i) ./ U(1, i); 
        y = U(v_deg2idx([0, 1]), i) ./ U(1, i); 
        
        if isinf(x) | isinf(y) | isnan(x) | isnan(x)
            continue; 
        end            
        
        f1 = sum((x .^ deg(1, :)) .* (y .^ deg(2, :)) .* (z .^ deg(3, :)) .* a); 
        f2 = sum((x .^ deg(1, :)) .* (y .^ deg(2, :)) .* (z .^ deg(3, :)) .* b); 
        [f1, f2, norm([x, y, z])]
%         (C{1} + C{2} * z + C{3} * z^2 + C{4} * z^3 + C{5} * z^4 + C{6} * z^5) * U(:, i)
%         break; 
        
    end
end
