
function [] = formation_control_circle(r, N, radius) 
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
    
    % Create the si_to_uni mapping function
    [si_to_uni_dyn] = create_si_to_uni_mapping3();
    
    % Generates the Weight Matrix for Noes 1:N-1 (i.e the moving nodes)
    W=zeros(N,N);
    for i= 1:N-1
        for j= 1:N-1
            % The weight to it's self should be zero
            if i == j
                W(i,j) = 0;
            % Case if the agent is less than pi radians around the circle
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

        % Get new robots' states
        xuni = r.get_poses(); 
        
        % Extract single integrator states
        dx=zeros(2,N);    
        
        % Initialize velocities to zero
        x = xuni(1:2,:);                                        

        % Generate xdot for all the agents
        for i= 1:N
            neighbors = topological_neighbors(L, i); 
            for j= neighbors
                dx(:,i) = dx(:,i) + (x(:,j) - x(:,i))*((norm(x(:,j) - x(:,i)) - W(i,j))/(norm(x(:,j) - x(:,i) + W(i,j))));
            end
        end
        
        % Convert single integrator inputs into unicycle inputs
        dx = si_to_uni_dyn(dx, xuni); 
        
        % Barrier certficate for using preventing crashing in robotarium
%         dx = si_barrier_certificate(dx, x);                    
       
        % Set new velocities to robots and update
        r.set_velocities(1:N, dx); r.step();              
       
        % Exit condition:
        %   Tests to see if all the weight conditions are met
        done = 0;
        for i=1:N-1
            for j=1:N-1
                % Checks the weight to the current position to see if all
                %   the agents met the desired formation
                % A small offset is given since the agent doesn't need to
                % be exactly on the point
                if norm(x(:,i)-x(:,j)) < W(i,j) - .001 || norm(x(:,i)-x(:,j)) > W(i,j) + .001
                     done = 1;                    
                end
            end  
        end
        
        % If all of the nodes are within the weight minus an offest then
        % formation is acheived!
        if done == 0
            result = 1;
        end
    end
 
disp('Done with formation')

end

function [theta] = cal_theta(i, j, N)
% Returns desired theta between nodes i and j on a circle given the number
% of nodes there should be
    theta = (pi/N)*mod(j-i,N);
end