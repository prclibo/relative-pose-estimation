d = [3, 3, 4, 4, 4, 5, 5, 6]; 

count = 0; 
fprintf('[ '); 
for i = 1:8
    fprintf('[ '); 
    for j = 1:8
        fprintf('c%d', count); 
        count = count + 1; 
        for k = 1:d(j)
            fprintf(' + c%d * z^%d', count, k); 
            count = count + 1; 
        end
        if j < 8 
            fprintf(', '); 
        end
    end
    fprintf(']'); 
    if i < 8
        fprintf(', '); 
    end
end
        
fprintf(' ]\n') ; 