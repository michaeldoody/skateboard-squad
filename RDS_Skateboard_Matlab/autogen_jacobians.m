function [dFK_com_b,dFK_com_t] = autogen_jacobians(boardDX,boardTheta,boardDTheta,bottomLinkRCoM,bottomLinkTheta,bottomLinkDTheta,bottomLinkHeight,topLinkRCoM,topLinkTheta,topLinkDTheta)
%AUTOGEN_JACOBIANS
%    [DFK_COM_B,DFK_COM_T] = AUTOGEN_JACOBIANS(BOARDDX,BOARDTHETA,BOARDDTHETA,BOTTOMLINKRCOM,BOTTOMLINKTHETA,BOTTOMLINKDTHETA,BOTTOMLINKHEIGHT,TOPLINKRCOM,TOPLINKTHETA,TOPLINKDTHETA)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    01-May-2020 09:32:28

t2 = boardTheta+bottomLinkTheta;
t3 = cos(t2);
t4 = sin(t2);
dFK_com_b = [boardDX-boardDTheta.*bottomLinkRCoM.*t3-bottomLinkRCoM.*bottomLinkDTheta.*t3;-boardDTheta.*bottomLinkRCoM.*t4-bottomLinkRCoM.*bottomLinkDTheta.*t4];
if nargout > 1
    t5 = t2+topLinkTheta;
    t6 = sin(t5);
    t7 = cos(t5);
    t8 = bottomLinkHeight.*t3;
    t9 = bottomLinkHeight.*t4;
    t10 = t7.*topLinkRCoM;
    t11 = t6.*topLinkRCoM;
    t12 = t8+t10;
    t13 = t9+t11;
    dFK_com_t = [boardDX-boardDTheta.*t12-bottomLinkDTheta.*t12-t10.*topLinkDTheta;-boardDTheta.*t13-bottomLinkDTheta.*t13-t11.*topLinkDTheta];
end
