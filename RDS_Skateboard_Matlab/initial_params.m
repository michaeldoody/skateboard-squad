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
    
    params.wheelRadius = 0.03;
    
% Bottom Link Parameters

    params.bottomLinkMass = 1.124;
    params.bottomLinkI = 0.0001; %% REPLACE
    params.bottomLinkTheta = 0; % wrt the skateboard, with 0 being positive x
    params.bottomLinkWidth = 0.04;
    params.bottomLinkHeight = .2;
    params.bottomLinkXCoM = 0;
    params.bottomLinkYCoM = .107; % m, taken wrt the joint with the skateboard
    
    
% Top Link Parameters

    params.topLinkMass = 2; % kg
    params.topLinkI = 0.0002; % REPLACE
    params.topLinkTheta = 0; % wrt the first link, with 0 being stright up (positive y)
    params.topLinkWidth = 0.085; % m, average of wide and narrow parts of top link
    params.topLinkHeight = .3;
    params.topLinkXCoM = 0;
    params.topLinkYCoM = 0.2;
    
    params.sim.dt = 0.03;
    dt = params.sim.dt;
    params.sim.tfinal = 3;
    tfinal = params.sim.tfinal;
    
    params.g = 5;
    
    params.bottomMotor.maxTorque = 30.6; % (Nm)
    params.topMotor.maxTorque = 16.2; % (Nm)
    
    params.bottomMotor.maxDTheta = 121.7 * 0.10472; % rpm converted to radians per second
    params.topMotor.maxDTheta = 243.5 * 0.10472; % rpm converted to radians per second
    
    params.bottomMotor.time = 0:dt:tfinal;
    params.topMotor.time = 0:dt:tfinal;
    
    params.bottomMotor.torque = zeros(1, length(0:params.sim.dt:params.sim.tfinal));
   % params.bottomMotor.torque(floor((length(params.bottomMotor.torque)/4)):floor(length(params.bottomMotor.torque)/2)) = .01;
   % params.bottomMotor.torque(ceil((length(params.bottomMotor.torque)/2)):ceil(3*length(params.bottomMotor.torque)/4)) = -.01;
    params.topMotor.torque = zeros(1, length(0:params.sim.dt:params.sim.tfinal));

    params.sim.constraints = ['false', 'false'];
    
    
    params.viz.colors.board = [1 0 0];
    params.viz.colors.bottomLink = [0 1 0];
    params.viz.colors.topLink = [0 0 1];
    params.viz.colors.boardCoM = [0 0.5 0.5];
    params.viz.colors.bottomLinkCoM = [0.75 0.75 0];
    params.viz.colors.topLinkCoM = [0.5 0 0.5];
    params.viz.colors.robotCoM = 'cyan';

    params.viz.axis_lims = [-1,1,-1,1];
  
end