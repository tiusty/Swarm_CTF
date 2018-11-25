
function [] = cyclicPursuit(r, N, radius, max_iter)
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
    [si_to_uni_dyn] = create_si_to_uni_mapping3();
    
    % Cyclic graph
    L = directedCircleGraphLaplacian(circularAgents);
    L = [L zeros(circularAgents,2)];
    L = [L ; zeros(2,N+1)];
    L(N,N) = 1;
    L(N,N+1)= -1;
    % Target cycle definition
    interAgentDistance = radius*2*sin(pi/circularAgents);
    kp1 = 7;
    kp2 = 0.4;
    xanchor = [-0.7;0.3];
    for k = 1:max_iter   
        
        % Get new data and initialize new null velocities
        xuni = r.get_poses();                                % Get new robots' states
        x = xuni(1:2,:);                                        % Extract single integrator states
        center = x(:,N);
        dx = zeros(2,N);                                           % Initialize velocities to zero         
        for i = 1:N               
            for j = topological_neighbors(L,i)
                if ~isempty(j)
                    if(i~= N)
                        alpha = pi/circularAgents + kp1*(interAgentDistance - norm(x(:,j)-x(:,i)) );
                        R = [cos(alpha), sin(alpha); -sin(alpha) cos(alpha)];
                        dx(:,i) = dx(:,i) + R*( x(:,j)-x(:,i) ) - kp2*(x(:,i) - center) + (norm(x(:, i) - center)^2 - radius^2)*(center - x(:, i));
                    else
                        dx(:,i) = .01*(xanchor-x(:,i));
                    end
                end
            end
        end
        dx = si_to_uni_dyn(dx, xuni);                            % Convert single integrator inputs into unicycle inputs
        r.set_velocities(1:N, dx); 
        r.step();
    end
end