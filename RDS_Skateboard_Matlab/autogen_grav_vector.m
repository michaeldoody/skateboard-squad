function G = autogen_grav_vector(bottomLinkRCoM,bottomLinkMass,bottomLinkTheta,bottomLinkHeight,g,topLinkRCoM,topLinkTheta)
%AUTOGEN_GRAV_VECTOR
%    G = AUTOGEN_GRAV_VECTOR(BOTTOMLINKRCOM,BOTTOMLINKMASS,BOTTOMLINKTHETA,BOTTOMLINKHEIGHT,G,TOPLINKRCOM,TOPLINKTHETA)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    01-May-2020 11:29:23

t2 = sin(bottomLinkTheta);
t3 = bottomLinkTheta+topLinkTheta;
t4 = sin(t3);
G = [0.0;-g.*(bottomLinkRCoM.*bottomLinkMass.*t2+bottomLinkMass.*bottomLinkHeight.*t2+bottomLinkMass.*t4.*topLinkRCoM);-bottomLinkMass.*g.*t4.*topLinkRCoM];
