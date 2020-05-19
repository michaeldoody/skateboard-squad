function [leftWheelDFK,rightWheelDFK] = autogen_wheel_velocities(boardDX,boardDY,boardTheta,boardDTheta,boardHeight,boardLength,wheelRadius)
%AUTOGEN_WHEEL_VELOCITIES
%    [LEFTWHEELDFK,RIGHTWHEELDFK] = AUTOGEN_WHEEL_VELOCITIES(BOARDDX,BOARDDY,BOARDTHETA,BOARDDTHETA,BOARDHEIGHT,BOARDLENGTH,WHEELRADIUS)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    18-May-2020 17:14:38

t2 = cos(boardTheta);
t3 = sin(boardTheta);
t4 = t2.*wheelRadius;
t5 = t3.*wheelRadius;
t6 = (boardHeight.*t2)./2.0;
t7 = (boardLength.*t2)./2.0;
t8 = (boardHeight.*t3)./2.0;
t9 = (boardLength.*t3)./2.0;
leftWheelDFK = [boardDX+boardDTheta.*(t4+t6+t9);boardDY+boardDTheta.*(t5-t7+t8)];
if nargout > 1
    rightWheelDFK = [boardDX+boardDTheta.*(t4+t6-t9);boardDY+boardDTheta.*(t5+t7+t8)];
end
