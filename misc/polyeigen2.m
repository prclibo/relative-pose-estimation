clc
clear all

d = 5; 
n = 56; 
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

s1_deg = zeros(3, 0); 
for i = 0:5
    for j = 0:5
        if i + j <=5
            s1_deg(:, end + 1) = [0; i; j]; 
        end
    end
end

s2_deg = zeros(3, 0); 
for i = 0:4
    for j = 0:5
        if i + j <=5
            s2_deg(:, end + 1) = [0; i; j]; 
        end
    end
end

s3_deg = zeros(3, 0); 
for i = 0:4
    for j = 0:4
        s3_deg(:, end + 1) = [0; i; j]; 
    end
end

deg2idx = @(x) find(deg(1, :) == x(1) & deg(2, :) == x(2) & deg(3, :) == x(3));

for i = 0:10
    for j = 0:10
        for k = 0:10
            if i + j + k <= 10 && isempty(deg2idx([i, j, k]))
                deg(:, end + 1) = [i; j; k]; 
            end
        end
    end
end

deg2idx = @(x) find(deg(1, :) == x(1) & deg(2, :) == x(2) & deg(3, :) == x(3));

c = zeros(1, n); 
c(deg2idx([2, 0, 0])) = 1; 
c(deg2idx([0, 2, 0])) = 1; 
c(deg2idx([0, 0, 2])) = 1; 
c(deg2idx([0, 0, 0])) = -1; 

M = zeros(0, 286); 
for i = 1:size(s1_deg, 2)
    M(end + 1, :) = 0; 
    for j = 1:56
        k = deg2idx(deg(:, j) + s1_deg(:, i)); 
        M(end, k) = a(j); 

    end
end

for i = 1:size(s2_deg, 2)
    M(end + 1, :) = 0; 
    for j = 1:56
        k = deg2idx(deg(:, j) + s2_deg(:, i)); 
        M(end, k) = b(j); 
    end
end

for i = 1:size(s3_deg, 2)
    M(end + 1, :) = 0; 
    for j = 1:56
        k = deg2idx(deg(:, j) + s3_deg(:, i)); 
        M(end, k) = c(j); 
    end
end



v_deg = deg(1:2, deg(3, :) == 0); 
m = size(v_deg, 2); 
C = cell(11, 1); 
for ci = 1:11
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

[U, D] = polyeig(C{1}, C{2}, C{3}, C{4}, C{5}, C{6}, C{7}, C{8}, C{9}, C{10}, C{11}); 

v_deg2idx = @(x) find(v_deg(1, :) == x(1) & v_deg(2, :) == x(2)); 

deg = deg(:, 1:56); 
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
