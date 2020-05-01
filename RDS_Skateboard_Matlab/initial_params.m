%% initial_params.m
%
% Description:
%   Initializes the values of many parameters, such as parameters in the
%   system dynamics, parameters that relate to simulating the system
%   forward in time, and parametes that relate to visualization/animation.
%   
% Inputs:
%   none

% Outputs:
%   params: a struct with many elements


function params = initial_params

% Skateboard Parameters

    params.boardMass = 0.85; % kg
    params.boardAngleInit = 0; % radians
    params.boardI = 0.00001; %% REPLACE
    params.boardXInit = 0;
    
    params.boardLength = 0.4;  % m
    params.boardHeight = 0.063;
    
% Bottom Link Parameters

    params.bottomLinkMass = 1.124;
    params.bottomLinkI = 0.0001; %% REPLACE
    params.bottomLinkTheta = 0; % wrt the skateboard, with 0 being positive x
    params.bottomLinkWidth = 0.04;
    params.bottomLinkHeight = .2;
    params.bottomLinkXCoM = 0;
    params.bottomLinkYCoM = .107; % m, taken wrt the joint with the skateboard
    
    
% Top Link Parameters

    params.topLinkMass = 1.498; % kg
    params.topLinkI = 0.0002; % REPLACE
    params.topLinkTheta = 0; % wrt the first link, with 0 being stright up (positive y)
    params.topLinkWidth = 0.085; % m, average of wide and narrow parts of top link
    params.topLinkHeight = .3;
    params.topLinkXCoM = 0;
    params.topLinkYCoM = 0.137;
    
    params.g = 9.8;
    
    params.sim.dt = 0.01;
    
    params.viz.colors.board = [1 0 0];
    params.viz.colors.bottomLink = [0 1 0];
    params.viz.colors.topLink = [0 0 1];
    params.viz.colors.bottomLinkCoM = [0.75 0.75 0.75];
    params.viz.colors.topLinkCoM = [0.5 0 0.5];
    params.viz.colors.tracers.boardCoM = [1 1 0];
    params.viz.colors.tracers.bottomLinkCoM = [1 0 1];
    params.viz.colors.tracers.topLinkCoM = [0 1 1];
    params.viz.axis_lims = [-5,5,-5,5];
  
end