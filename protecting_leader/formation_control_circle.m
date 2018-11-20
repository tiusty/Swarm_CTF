
function [] = formation_control_circle(N, rbtm, si_to_uni_dyn) 
%   Runs formation control on a graph to put the agents in a circle
%   around node N (which goes to (0,0)
%
%   Arguments:
%       N: (int) -> The number of nodes including the center agent in the
%       graph
%       rbtm: The rb.set_number_of_agents call from the
%           robotarium
%       si_to_uni_dyn: The function that does conversion from uni-cycle to
%           single integrator dynamics
   
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

    result = 0;
    while( result == 0)

        % Get new data and initialize new null velocities
        xuni = rbtm.get_poses();                                % Get new robots' states
        x = xuni(1:2,:);                                        % Extract single integrator states

        dx=zeros(2,N);                                           % Initialize velocities to zero

        % FILL THIS PART!!!
        for i= 1:N
            neighbors = topological_neighbors(L, i); 
            for j= neighbors
                dx(:,i) = dx(:,i) + (x(:,j) - x(:,i))*((norm(x(:,j) - x(:,i)) - W(i,j))/(norm(x(:,j) - x(:,i) + W(i,j))));
            end
            sum(dx)
        end

        dx = si_to_uni_dyn(dx, xuni);                            % Convert single integrator inputs into unicycle inputs
    %     dx = si_barrier_certificate(dx, x);                    
        rbtm.set_velocities(1:N, dx); rbtm.step();              % Set new velocities to robots and update
        if all(abs(sum(dx)) < .3)
            result = 1;
        end
        sum(dx)
    end
 
disp('Down with forming circle')

end

function [theta] = cal_theta(i, j, N)
    theta = (pi/N)*mod(j-i,N);
end