function [A_all_ramp,H_constL_ramp,H_constR_ramp] = autogen_constraint_derivatives_ramp(boardTheta,boardHeight,boardLength,normLeftX,normLeftY,normRightX,normRightY,wheelRadius)
%AUTOGEN_CONSTRAINT_DERIVATIVES_RAMP
%    [A_ALL_RAMP,H_CONSTL_RAMP,H_CONSTR_RAMP] = AUTOGEN_CONSTRAINT_DERIVATIVES_RAMP(BOARDTHETA,BOARDHEIGHT,BOARDLENGTH,NORMLEFTX,NORMLEFTY,NORMRIGHTX,NORMRIGHTY,WHEELRADIUS)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    02-Jun-2020 04:02:20

t2 = cos(boardTheta);
t3 = sin(boardTheta);
t4 = t2.*wheelRadius;
t5 = t3.*wheelRadius;
t6 = (boardHeight.*t2)./2.0;
t7 = (boardLength.*t2)./2.0;
t8 = (boardHeight.*t3)./2.0;
t9 = (boardLength.*t3)./2.0;
t10 = -t7;
t11 = -t9;
t12 = t4+t6+t9;
t13 = t5+t7+t8;
t14 = t4+t6+t11;
t15 = t5+t8+t10;
A_all_ramp = reshape([normLeftX,normRightX,normLeftY,normRightY,normLeftX.*t12+normLeftY.*t15,normRightX.*t14+normRightY.*t13,0.0,0.0,0.0,0.0],[2,5]);
if nargout > 1
    H_constL_ramp = reshape([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,normLeftY.*t12-normLeftX.*t15,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],[5,5]);
end
if nargout > 2
    H_constR_ramp = reshape([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-normRightX.*t13+normRightY.*t14,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],[5,5]);
end
