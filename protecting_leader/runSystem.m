clear all, close all, clc
% Script runs the formation control and cyclic pursuit together

% Number of agents
N=7;

flag = [.5;0];
base = [-.5; 0];

% Number of iterations to run
max_iter = 5000;           

% The radius of the formation and the radius to run cyclic pursuit
radius = .5;

% Initialize robotarium
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true);
    
% Create plot handle for flag
flag_plot = plot([0,0], [0,0], '-db');

% Center node attempts to find the flag
findFlag(r, N, flag, flag_plot)

% Run the formation control
formationControlCircle(r, N, radius, flag, flag_plot)

% Run the cyclic Pursuit
cyclicPursuit(r, N, radius, max_iter, base, flag_plot)

% We can call this function to debug our experiment!  Fix all the errors
% before submitting  to maximize the chance that your experiment runs
% successfully.
r.debug();