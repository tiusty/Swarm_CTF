clear all, close all, clc



N = 8;                        % Number of agents
dt=0.001;                   % numerical steplength
max_iter = 3000;

videoFLag = 0;                          % Change to 1 to record video
radius = .5;                           

% Initialize robotarium
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true);

[si_to_uni_dyn] = create_si_to_uni_mapping3();

% Initialize robots
xuni = r.get_poses();                                    % States of real unicycle robots
x = xuni(1:2,:);                                            % x-y positions only
r.set_velocities(1:N, zeros(2,N));                       % Assign dummy zero velocity
r.step();                                                % Run robotarium step
 
circularAgents = N-1;
circularTargets = radius *[ cos(0:2*pi/circularAgents:2*pi*(1- 1/circularAgents)) 0;sin(0:2*pi/circularAgents:2*pi*(1- 1/circularAgents)) 0];
errorToInitialPos = x - circularTargets;                % Error
errorNorm = [1,1]*(errorToInitialPos.^2);               % Norm of error
while max( errorNorm ) > 0.03
        % Update state variables       0 
    xuni = r.get_poses();                            % States of real unicycle robots
    x = xuni(1:2,:);                                    % x-y positions      
        % Update errors
    errorToInitialPos = x - circularTargets;
    errorNorm = [1,1]*(errorToInitialPos.^2);
        
    % Conput control inputs
    u = -0.3.*errorToInitialPos;
    dx = si_to_uni_dyn(u, xuni);
        
    % Assing new control inputs to robots
    r.set_velocities(1:N, dx);                       % Assign dummy zero velocity
    r.step();                                        % Run robotarium step
end
disp('Initial positions reached')


% Initialize video
if videoFLag 
    vid = VideoWriter('HW4_formaionControl.mp4', 'MPEG-4');
    vid.Quality = 100;
    vid.FrameRate = 72;
    open(vid);
    writeVideo(vid, getframe(gcf));
end
  
cyclicPursuit(r, N, radius, max_iter)
% Though we didn't save any data, we still should call r.call_at_scripts_end() after our
% experiment is over!
r.call_at_scripts_end();

