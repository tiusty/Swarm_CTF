
function [] = cyclicPursuit(r, N, radius, max_iter , xanchor, flag_plot)
%   Runs cyclic pursuit so that the agents spin around the center
%       agent evenly spaced. Will form to evenly spaced if it does
%       not start evenly spaced
%
%   Arguments:
%       r: The robotarium object
%       N: (int) -> The number of nodes including the center agent in the
%       graph
%       radius: (int) -> The radius that the nodes attempt to form for the circle
%       max_iter: (int) -> The number of iterations to run the cyclic
%       pursuit for

    circularAgents = N-1;
    
    % Create the si_to_uni mapping function
    [si_to_uni_dyn] = create_si_to_uni_mapping3();
    
    % Create barrier certificate fucntion
    si_barrier_certificate = create_si_barrier_certificate('SafetyRadius', 1.5*r.robot_diameter);
    
    
    % Cyclic graph
    L = directedCircleGraphLaplacian(circularAgents);
    L = [L zeros(circularAgents,2)];
    L = [L ; zeros(2,N+1)];
    

    
    L(N,N) = 1;
    L(N,N+1)= -1;
    
    % Target cycle definition
    interAgentDistance = radius*2*sin(pi/circularAgents);
    kp1 = 8;
    kp2 = 0.4;
    
    % Agent label
    plTx = cell(N,1);
    for i=1:N 
        plTx{i} = text(-100,-100,num2str(i)); 
    end

    
    for k = 1:max_iter   
        
        % Get new data and initialize new null velocities
        xuni = r.get_poses();                                % Get new robots' states
        x = xuni(1:2,:);                                        % Extract single integrator states
        center = x(:,N);
        dx = zeros(2,N);  
        
        if (k == 1)
            theta = atan2(x(2,1) - center(2), x(1,1) -center(1));
    
            x_loc = radius * cos(theta + 2*pi/(N-1)) + center(1);
            y_loc = radius * sin(theta + 2*pi/(N-1)) + center(2);

            dist_2 = norm(x(:,2) - [x_loc, y_loc]);
            dist_6 = norm(x(:,6) - [x_loc, y_loc]);

            if (dist_6 < dist_2)
               L = transpose(L);
            end 
            
            L(N,N) = 1;
            L(N,N+1)= -1;
        end
        
        
        % Initialize velocities to zero         
        for i = 1:N               
            for j = topological_neighbors(L,i)
                if ~isempty(j)
                    if(i~= N)
                        alpha = pi/circularAgents + kp1*(interAgentDistance - norm(x(:,j)-x(:,i)) );
                        R = [cos(alpha), sin(alpha); -sin(alpha) cos(alpha)];
                        dx(:,i) = dx(:,i) + R*( x(:,j)-x(:,i) ) - kp2*(x(:,i) - center) + (norm(x(:, i) - center)^2 - radius^2)*(center - x(:, i));
                    else
                        
                        if (k > 200)                        
                            dx(:,i) = .01*(xanchor-x(:,i));
                        end
                    end
                end
            end
        end
        
        % To avoid errors, we need to threshold dx
        norms = arrayfun(@(x) norm(dx(:, x)), 1:N);
        threshold = r.max_linear_velocity/2;
        to_thresh = norms > threshold;
        dx(:, to_thresh) = threshold*dx(:, to_thresh)./norms(to_thresh);
        
        % Barrier certficate for using preventing crashing in robotarium
        dx = si_barrier_certificate(dx, xuni);
        
        % Convert single integrator inputs into unicycle inputs
        dx = si_to_uni_dyn(dx, xuni); 
      
        % Set new velocities to robots and update
        r.set_velocities(1:N, dx); r.step();   
        
        flag_plot.XData = x(1,N);
        flag_plot.YData = x(2,N);
        
        for i = 1:N 
            set(plTx{i},'position',[x(1,i)+0.05,x(2,i)+0.05])
        end
        
        
    end
end

function [theta] = cal_theta(i, j, N)
% Returns desired theta between nodes i and j on a circle given the number
% of nodes there should be
%   Arguments:
%       i: (int) ->  Node i: The first node 
%       j: (int) -> Node j: The second node
%       graph
%       N: (int) -> The number of nodes including the center agent in the
%       graph
    theta = (pi/N)*mod(j-i,N);
end