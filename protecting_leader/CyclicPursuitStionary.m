clear all, close all, clc

N=10;                        % Number of agents
dt=0.01;                   % numerical steplength
max_iter = 10000;                           

% Initialize robotarium
rb = RobotariumBuilder();
rbtm = rb.set_number_of_agents(N).set_save_data(false).build();
[si_to_uni_dyn] = create_si_to_uni_mapping3();

% Initialize robots
xuni = rbtm.get_poses();                                    % States of real unicycle robots
x = xuni(1:2,:);                                            % x-y positions only
rbtm.set_velocities(1:N, zeros(2,N));                       % Assign dummy zero velocity
rbtm.step();                                                % Run robotarium step

% Complete graph
L = completeGL(N);
% L(N,:) = 0; % This makes the center node stationary

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
    xuni = rbtm.get_poses();                                % Get new robots' states
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
    rbtm.set_velocities(1:N, dx); rbtm.step();              % Set new velocities to robots and update
    
end

% Though we didn't save any data, we still should call r.call_at_scripts_end() after our
% experiment is over!
rbtm.call_at_scripts_end();

function [theta] = cal_theta(i, j, N)
    theta = (pi/N)*mod(j-i,N);
end
