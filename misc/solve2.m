clc
clear all

d = 5; 
n = 56; 
m = 21; 

deg = [4   0    1
    3	1	1
    2	2	1
    2	1	2
    1	2	2
    0	4	1
    4	0	0
    3	1	0
    2	2	0
    2	1	1
    1	2	1
    0	4	0
    2	1	0
    1	2	0
    0	5	0
    1	3	0
    1	3	1
    1	4	0
    2	3	0
    3	2	0
    4	1	0
    5	0	0
    0	0	0
    0	0	1
    0	0	2
    0	0	3
    0	0	4
    0	0	5
    0	1	0
    0	1	1
    0	1	2
    0	1	3
    0	1	4
    0	2	0
    0	2	1
    0	2	2
    0	2	3
    0	3	0
    0	3	1
    0	3	2
    1	0	0
    1	0	1
    1	0	2
    1	0	3
    1	0	4
    1	1	0
    1	1	1
    1	1	2
    1	1	3
    2	0	0
    2	0	1
    2	0	2
    2	0	3
    3	0	0
    3	0	1
    3	0	2]';

a = rand(1, n); 
b = rand(1, n); 


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

M = rref(M); 


extra_deg = zeros(3, 0); 
for i = 1:56
    if isempty(deg2idx(deg(:, i) + [0; 0; 1]))
        extra_deg(:, end + 1) = deg(:, i) + [0; 0; 1]; 
    end
end

deg = [deg, extra_deg]; 
deg2idx = @(x) find(deg(1, :) == x(1) & deg(2, :) == x(2) & deg(3, :) == x(3)); 
B = zeros(0, size(deg, 2)); 

for row0 = 7:14
    col0 = row0; 
    col1 = deg2idx(deg(:, col0) + [0; 0; 1]); 
    row1 = col1; 
    
    B(end + 1, :) = 0; 
    B(end, 1:56) = M(row1, :); 
    for j = 1:56
        k = deg2idx(deg(:, j) + [0; 0; 1]); 
        B(end, k) = B(end, k) - M(row0, j); 
    end
end


