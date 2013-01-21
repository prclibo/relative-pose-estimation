clc
f = fopen('eq6.txt'); 

a = 1; 

coef = cell(1); 
while ~feof(f)
    c = fscanf(f, '%c', 1); 
    if c == 10 
        continue; 
    end
    if c == '(' 
        break;
    else
        coef{1}(end + 1) = c; 
    end
end

coef{1} = coef{1}(1:end - 1); 
fprintf('a[0] + '); 

while ~feof(f)
    if c == 10
        c = fscanf(f, '%c', 1); 
        continue; 
    elseif c == '('
        coef{end + 1} = []; 
        c = fscanf(f, '%c', 1); 
        while c ~= ')'
            if c == 10 
                c = fscanf(f, '%c', 1); 
                continue; 
            end
            coef{end}(end + 1) = c; 
            c = fscanf(f, '%c', 1); 
        end
%         fprintf(f, 'k[%d]', a); 
        fprintf(' a[%d]', a); 
        a = a + 1; 
    else
%         fprintf(f, '%c', c); 
        fprintf('%c', c); 
    end
    c = fscanf(f, '%c', 1); 
end
fprintf('\n'); 
for i = 1:length(coef)
    coef{i}(coef{i} == '\') = []; 
    fprintf('a[%d] = %s\n', i - 1, coef{i}); 
end

%%

f = fopen('eq7.txt'); 

a = 1; 

coef = cell(1); 
while ~feof(f)
    c = fscanf(f, '%c', 1); 
    if c == 10 
        continue; 
    end
    if c == '(' 
        break;
    else
        coef{1}(end + 1) = c; 
    end
end

coef{1} = coef{1}(1:end - 1); 
fprintf('b[0] + '); 

while ~feof(f)
    if c == 10
        c = fscanf(f, '%c', 1); 
        continue; 
    elseif c == '('
        coef{end + 1} = []; 
        c = fscanf(f, '%c', 1); 
        while c ~= ')'
            if c == 10 
                c = fscanf(f, '%c', 1); 
                continue; 
            end
            coef{end}(end + 1) = c; 
            c = fscanf(f, '%c', 1); 
        end
%         fprintf(f, 'k[%d]', a); 
        fprintf(' b[%d]', a); 
        a = a + 1; 
    else
%         fprintf(f, '%c', c); 
        fprintf('%c', c); 
    end
    c = fscanf(f, '%c', 1); 
end
fprintf('\n'); 
for i = 1:length(coef)
    coef{i}(coef{i} == '\') = []; 
    fprintf('b[%d] = %s\n', i - 1, coef{i}); 
end
