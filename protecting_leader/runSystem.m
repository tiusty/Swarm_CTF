clear all, close all, clc
% Script runs the formation control and cyclic pursuit together

% Number of agents
N=7;     

% Number of iterations to run
max_iter = 1000;           

% The radius of the formation and the radius to run cyclic pursuit
radius = .5;

% Initialize robotarium
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true);

% Run the formation control
formationControlCircle(r, N, radius)

% Run the cyclic Pursuit
cyclicPursuit(r, N, radius, max_iter)

% We can call this function to debug our experiment!  Fix all the errors
% before submitting to maximize the chance that your experiment runs
% successfully.
r.debug();



