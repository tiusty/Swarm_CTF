
function [] = formation_control_circle(N, rbtm, si_to_uni_dyn) 
%   Runs formation control on a graph to put the agents in a circle
%   around node N. Currently node N also moves to fit the formation
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

    % Initial Arguments
    radius = .5;
    
    % Generates the Weight Matrix for Noes 1:N-1 (i.e the moving nodes)
    W=zeros(N,N);
    for i= 1:N-1
        for j= 1:N-1
            if i == j
                W(i,j) = 0;
            % Care if the agent is less than pi radians around the circle
            elseif cal_theta(i,j , N-1) < pi/2
                W(i,j) = 2* radius*sin(cal_theta(i,j, N-1));
            % Case if the agents are more than pi radians around the circle
            elseif cal_theta(i,j, N-1) > pi/2
                W(i,j) = 2* radius*sin(pi - cal_theta(i,j, N-1));
            % Case if the agents are directly across from each other
            elseif cal_theta(i,j, N-1) == pi/2
                W(i,j) = 2 * radius;
            end  
        end
    end
    
    % Set the weight matrix values for the center node
    W(:,N) = radius;
    W(N,:) = radius;

    % Flag variable to test for condition is reached
    result = 0;
    
    % Loop until the formation is achieved
    while( result == 0)

        % Get new data and initialize new null velocities
        
        % Get new robots' states
        xuni = rbtm.get_poses(); 
        % Extract single integrator states
        dx=zeros(2,N);    
        % Initialize velocities to zero
        x = xuni(1:2,:);                                        

        % Generate xdot
        for i= 1:N
            neighbors = topological_neighbors(L, i); 
            for j= neighbors
                dx(:,i) = dx(:,i) + (x(:,j) - x(:,i))*((norm(x(:,j) - x(:,i)) - W(i,j))/(norm(x(:,j) - x(:,i) + W(i,j))));
            end
        end

        dx = si_to_uni_dyn(dx, xuni);                            % Convert single integrator inputs into unicycle inputs
%         dx = si_barrier_certificate(dx, x);   % Needed in robotarium                 
        rbtm.set_velocities(1:N, dx); rbtm.step();              % Set new velocities to robots and update
       
        % Exit condition, currently achieved is all nodes are close to not
        % moving
        if all(abs(sum(dx)) < .3)
            result = 1;
        end
    end
 
disp('Done with forming circle')

end

function [theta] = cal_theta(i, j, N)
% Returns desired theta between nodes i and j on a circle given the number
% of nodes there should be
    theta = (pi/N)*mod(j-i,N);
end