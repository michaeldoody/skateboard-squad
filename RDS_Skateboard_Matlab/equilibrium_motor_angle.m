%% equilibrium_motor_angle.m
%
% Description:
%   Wrapper function for autogen_equilibrium_motor_angle.m
%   Computes the body motor rotation needed to put the robot in an
%   equilibrium configuration, given boardTheta, bottomLinkTheta, and
%   topLinkTheta
%
% Inputs:
%   x: the state vector, x = [q; q_dot];
%   params: a struct with many elements, generated by calling init_params.m
%
% Outputs:
%   theta_m_eq = values of motor angle that establishes equilibrium

function [theta_m_eq] = equilibrium_motor_angle(x,params)

boardMass = params.boardMass;
boardTheta = x(3);
boardHeight = params.boardHeight/2;  % divide by 2 because jumping robot example measures height to center of mass
boardLength = params.boardLength/2;
bottomLinkMass = params.bottomLinkMass;
bottomLinkTheta = x(4);
r = params.wheelRadius;
topLinkMass = params.topLinkMass;

% topLinkTheta = x(5);
% bottomLinkMass = params.bottomLinkMass;
% topLinkMass = params.topLinkMass;
% wheelRadius = params.wheelRadius;
% bottomLinkHeight =  params.bottomLinkHeight;

theta_m_eq = autogen_equilibrium_motor_angle(boardMass,boardTheta,boardHeight,boardLength,bottomLinkMass,bottomLinkTheta,r,topLinkMass);

% theta_m_eq = autogen_equilibrium_motor_angle(boardHeight,bottomLinkHeight,boardMass,bottomLinkMass,topLinkMass,wheelRadius,boardTheta,bottomLinkTheta,topLinkTheta,boardLength);
end