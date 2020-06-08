%% pid_idle.m
%
% Description:
%  Computes the controls in order to maintain a robot's position 
%  steady while it moves along the ramp.
%
% Inputs:
%   bottomAngle: current angle of the bottom link
%   topAngle: current angle of the top link
%   desiredBottom: desired angle for the bottom link
%   desiredTop: desired angle for the top link

%
% Outputs:
%   bottomControl, topControl = control torques for both motors

function [bottomControl, topControl] = ...
          pid_idle(bottomAngle, topAngle, desiredBottom, desiredTop) 
      
global prevBottomErrorIdle
global prevTopErrorIdle

      
bottomError = desiredBottom - bottomAngle;
topError = desiredTop - topAngle;

kp_bottom = 50;
kd_bottom = 80000/3;


kp_top = 50;
kd_top = 80000/3;


bottomControl = kp_bottom * bottomError + (bottomError - prevBottomErrorIdle) * kd_bottom;
topControl = kp_top * topError + (topError - prevTopErrorIdle) * kd_top;


prevBottomErrorIdle = bottomError;
prevTopErrorIdle = topError;


end

