clear all, close all, clc

% Number of agents
N=10;     

% Number of iterations to run
max_iter = 10000;           

% The radius of the formation and the radius to run cyclic pursuit
radius = .5;

% Initialize robotarium
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true);

% Run the formation control
formationControlCircle(r, N, radius)

% Run the cyclic Pursuit
cyclicPursuit(r, N, radius, max_iter)

% Though we didn't save any data, we still should call r.call_at_scripts_end() after our
% experiment is over!
rbtm.call_at_scripts_end();


