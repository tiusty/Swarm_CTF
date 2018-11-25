clear all, close all, clc

N=10;                        % Number of agents
dt=0.01;                   % numerical steplength
max_iter = 10000;   
radius = .5;

% Initialize robotarium
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true);

% Initialize robots
% xuni = r.get_poses();                                    % States of real unicycle robots
% x = xuni(1:2,:);                                            % x-y positions only
% r.set_velocities(1:N, zeros(2,N));                       % Assign dummy zero velocity
% r.step();                                                % Run robotarium step

% Run the formation control
formation_control_circle(r, N, radius)

% Run the cyclic Pursuit
cyclicPursuit(r, N, radius, max_iter)

% Though we didn't save any data, we still should call r.call_at_scripts_end() after our
% experiment is over!
rbtm.call_at_scripts_end();


