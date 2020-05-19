function [bottomControl, topControl] = ...
          pid_angle(bottomAngle, desiredBottomAngle, topAngle, desiredTopAngle) 
                                            

bottomError = desiredBottomAngle - bottomAngle;
topError = desiredTopAngle - topAngle;

kp_bottom = 20;

kp_top = 20;



bottomControl = kp_bottom * bottomError;
topControl = kp_top * topError;


end

