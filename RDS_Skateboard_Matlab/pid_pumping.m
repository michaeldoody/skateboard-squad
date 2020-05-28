function [bottomControl, topControl] = ...
          pid_pumping(bottomLinkTheta, topLinkTheta) 
      
global prevBottomErrorPumping
global prevTopErrorPumping


desiredBLTheta = -0.7;
desiredTLTheta = -2*desiredBLTheta;
    

bottomError =  desiredBLTheta - bottomLinkTheta;
topError = desiredTLTheta - topLinkTheta;

kp_bottom = 50;
kd_bottom = 3000;


kp_top = 100;
kd_top = 8000;


bottomControl = kp_bottom * bottomError + (bottomError - prevBottomErrorPumping) * kd_bottom;
topControl = kp_top * topError + (topError - prevTopErrorPumping) * kd_top;


prevBottomErrorPumping = bottomError;
prevTopErrorPumping = topError;


end

