clear all, close all, clc

N=10;                        % Number of agents
dt=0.01;                   % numerical steplength
max_iter = 10000;                           

% Initialize robotarium
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true);

[si_to_uni_dyn] = create_si_to_uni_mapping3();

% Initialize robots
xuni = r.get_poses();                                    % States of real unicycle robots
x = xuni(1:2,:);                                            % x-y positions only
r.set_velocities(1:N, zeros(2,N));                       % Assign dummy zero velocity
r.step();                                                % Run robotarium step

formation_control_circle(N, r , si_to_uni_dyn)

% Cyclic graph
% A = diag(ones(N-1,1),-1);
% A(1,N) = 1; 
% A(N,1)
% L = diag(sum(A)) - A;
L = [ 1 0 -1 0;
    -1 1 0 0;
    0 -1 1 0;
    0 0 0 0]

% Target cycle definition
center = [0;0];
radius = 0.7;
interAgentDistance = radius*2*sin(pi/N);
kp1 = 7;
kp2 = 0.4;
plot(center(1),center(2),'*','markersize',12)
th = 0 : 2*pi/20 : 2*pi-2*pi/20;
plot(radius.*cos(th)+center(1),radius.*sin(th)+center(2),'b')
  
for k = 1:max_iter
    
    % Get new data and initialize new null velocities
    xuni = r.get_poses();                                % Get new robots' states
    x = xuni(1:2,:);                                        % Extract single integrator states

    dx = zeros(2,N);                                           % Initialize velocities to zero         
    for i = 1:N-1                
         for j = topological_neighbors(L,i)
            if ~isempty(j)
                alpha = pi/N + kp1*(interAgentDistance - norm(x(:,j)-x(:,i)) );
                R = [cos(alpha), sin(alpha); -sin(alpha) cos(alpha)];
                dx(:,i) = dx(:,i) + R*( x(:,j)-x(:,i) ) - kp2*( x(:,i) - center );
            end
        end
    end
    

    dx = si_to_uni_dyn(dx, xuni);                            % Convert single integrator inputs into unicycle inputs
    
    % Set velocities of agents 1,...,N
    r.set_velocities(1:N, dx);

    % Send the previously set velocities to the agents.  This function must be called!
    r.step();    
    
end

% Though we didn't save any data, we still should call r.call_at_scripts_end() after our
% experiment is over!
rbtm.call_at_scripts_end();


