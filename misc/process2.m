clc
f = fopen('eq10.txt'); 

a = 1; 

coef = cell(1); 
while ~feof(f)
    c = fscanf(f, '%c', 1); 
    if c == '(' 
        break;
    else
        coef{1}(end + 1) = c; 
    end
end

coef{1} = coef{1}(1:end - 1); 
fprintf('c[0] + '); 

while ~feof(f)
    if c == '('
        coef{end + 1} = []; 
        c = fscanf(f, '%c', 1); 
        while c ~= ')'
            coef{end}(end + 1) = c; 
            c = fscanf(f, '%c', 1); 
        end
%         fprintf(f, 'k[%d]', a); 
        fprintf(' c[%d]', a); 
        a = a + 1; 
    else
%         fprintf(f, '%c', c); 
        fprintf('%c', c); 
    end
    c = fscanf(f, '%c', 1); 
end

%%


f = fopen('eq11.txt'); 

a = 1; 

coef = cell(1); 
while ~feof(f)
    c = fscanf(f, '%c', 1); 
    if c == '(' 
        break;
    else
        coef{1}(end + 1) = c; 
    end
end

coef{1} = coef{1}(1:end - 1); 
fprintf('d[0] + '); 

while ~feof(f)
    if c == '('
        coef{end + 1} = []; 
        c = fscanf(f, '%c', 1); 
        while c ~= ')'
            coef{end}(end + 1) = c; 
            c = fscanf(f, '%c', 1); 
        end
%         fprintf(f, 'k[%d]', a); 
        fprintf(' d[%d]', a); 
        a = a + 1; 
    else
%         fprintf(f, '%c', c); 
        fprintf('%c', c); 
    end
    c = fscanf(f, '%c', 1); 
end