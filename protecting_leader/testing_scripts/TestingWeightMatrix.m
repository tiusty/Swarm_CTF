clear all
N=8;

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
        elseif cal_theta(i,j , N-1) < pi/2
            W(i,j) = 2* radius*sin(cal_theta(i,j, N-1));
        elseif cal_theta(i,j, N-1) > pi/2
            W(i,j) = 2* radius*sin(pi - cal_theta(i,j, N-1));
        elseif cal_theta(i,j, N-1) == pi/2
            W(i,j) = 2 * radius;
        end  
    end
end
W(:,N) = radius;
W(N,:) = radius;
W

function [theta] = cal_theta(i, j, N)
    theta = (pi/N)*mod(j-i,N);
end

