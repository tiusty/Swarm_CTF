function [L] = directedCircleGraphLaplacian(N)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
D = diag(ones(1,N));
cycle_adjancy_row = zeros(1,N);
cycle_adjancy_row(2) = 1;
A = [];
A = [A;cycle_adjancy_row];
for i=1:N-1
    cycle_adjancy_row = circshift(cycle_adjancy_row,1);
    A = [A;cycle_adjancy_row];
end

L = D-A;
end

