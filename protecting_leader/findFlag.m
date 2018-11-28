
function [] = findFlag(r, N, flag) 
% Loop until the formation is achieved
    % Create the si_to_uni mapping function
    [si_to_uni_dyn] = create_si_to_uni_mapping3();
    
    % Create barrier certificate fucntion
    si_barrier_certificate = create_si_barrier_certificate('SafetyRadius', 1.5*r.robot_diameter);
    result = 0;
    while( result == 0)

        % Get new robots' states
        xuni = r.get_poses(); 
        
        % Extract single integrator states
        dx=zeros(2,N);    
        
        % Initialize velocities to zero
        x = xuni(1:2,:); 
        d = .5;

        % Generate xdot for all the agents
        for i= 1:N-1
            neighbors = delta_disk_neighbors(x, i, d);
            for j= neighbors
                if j == N
                    dx(:,i) = dx(:,i) + (x(:,j) - x(:,i) - d);
                end
            end
        end
        
        % Send the center node to (0,0) to prevent out of boundary error
        dx(:,N) = flag - x(:,N);
        
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
 
        % If all of the nodes are within the weight minus an offest then
        % formation is acheived!
        if norm(x(:,N) - flag) < .2
            result = 1;
        end
       
    end
end