clear all
N=7;

% Complete graph
L = completeGL(N);

radius = .5;
id = 2 * (radius*sin(pi/(N-1)));
ido = 2* radius*sin((pi/(N-1))*floor((N-1)/2)); % Opposite 
W=zeros(N,N);
for i= 1:N-1
    for j= 1:N-1
        if i == j
            W(i,j) = 0;
        elseif i == mod(j + 1, N-1) || (mod(j + 1, N-1) == 0 && i == N-1) || i == mod(j - 1, N-1) || (mod(j - 1, N-1) == 0 && i == N-1)
                W(i,j) = id;
        elseif i == mod(j - floor((N-1)/2), N-1) || (mod(j - floor((N-1)/2), N-1) == 0 && i == N-1) || i == mod(j - ceil((N-1)/2), N-1) || (mod(j - ceil((N-1)/2), N-1) == 0 && i == N-1) 
            W(i,j) = ido;
        else
            W(i,j) = 0;
        end  
    end
end
W(:,N) = radius;
W(N,:) = radius;
W
