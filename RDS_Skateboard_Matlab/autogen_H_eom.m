function H = autogen_H_eom(boardMass,boardTheta,boardDTheta,boardHeight,bottomLinkRCoM,bottomLinkMass,bottomLinkTheta,bottomLinkDTheta,bottomLinkHeight,g,topLinkRCoM,topLinkMass,topLinkTheta,topLinkDTheta)
%AUTOGEN_H_EOM
%    H = AUTOGEN_H_EOM(BOARDMASS,BOARDTHETA,BOARDDTHETA,BOARDHEIGHT,BOTTOMLINKRCOM,BOTTOMLINKMASS,BOTTOMLINKTHETA,BOTTOMLINKDTHETA,BOTTOMLINKHEIGHT,G,TOPLINKRCOM,TOPLINKMASS,TOPLINKTHETA,TOPLINKDTHETA)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    05-Jun-2020 09:07:55

t2 = cos(boardTheta);
t3 = sin(boardTheta);
t4 = sin(bottomLinkTheta);
t5 = sin(topLinkTheta);
t6 = boardTheta+bottomLinkTheta;
t7 = bottomLinkTheta+topLinkTheta;
t8 = boardDTheta.^2;
t9 = bottomLinkDTheta.^2;
t10 = topLinkDTheta.^2;
t11 = cos(t6);
t12 = sin(t6);
t13 = t6+topLinkTheta;
t14 = sin(t7);
t19 = bottomLinkHeight.*t5.*t10.*topLinkRCoM.*topLinkMass;
t20 = boardDTheta.*bottomLinkHeight.*t5.*topLinkRCoM.*topLinkMass.*topLinkDTheta.*2.0;
t21 = bottomLinkDTheta.*bottomLinkHeight.*t5.*topLinkRCoM.*topLinkMass.*topLinkDTheta.*2.0;
t15 = sin(t13);
t16 = cos(t13);
t17 = bottomLinkRCoM.*bottomLinkMass.*g.*t12;
t18 = bottomLinkHeight.*g.*t12.*topLinkMass;
t25 = -t20;
t26 = -t21;
t27 = -t19;
t22 = g.*t15.*topLinkRCoM.*topLinkMass;
t23 = -t17;
t24 = -t18;
t28 = -t22;
H = [(boardHeight.*bottomLinkMass.*t3.*t8)./2.0+bottomLinkRCoM.*bottomLinkMass.*t8.*t12+bottomLinkRCoM.*bottomLinkMass.*t9.*t12+(boardHeight.*t3.*t8.*topLinkMass)./2.0+bottomLinkHeight.*t8.*t12.*topLinkMass+bottomLinkHeight.*t9.*t12.*topLinkMass+t8.*t15.*topLinkRCoM.*topLinkMass+t9.*t15.*topLinkRCoM.*topLinkMass+t10.*t15.*topLinkRCoM.*topLinkMass+boardDTheta.*bottomLinkRCoM.*bottomLinkMass.*bottomLinkDTheta.*t12.*2.0+boardDTheta.*bottomLinkDTheta.*bottomLinkHeight.*t12.*topLinkMass.*2.0+boardDTheta.*bottomLinkDTheta.*t15.*topLinkRCoM.*topLinkMass.*2.0+boardDTheta.*t15.*topLinkRCoM.*topLinkMass.*topLinkDTheta.*2.0+bottomLinkDTheta.*t15.*topLinkRCoM.*topLinkMass.*topLinkDTheta.*2.0;boardMass.*g+bottomLinkMass.*g+g.*topLinkMass-(boardHeight.*bottomLinkMass.*t2.*t8)./2.0-bottomLinkRCoM.*bottomLinkMass.*t8.*t11-bottomLinkRCoM.*bottomLinkMass.*t9.*t11-(boardHeight.*t2.*t8.*topLinkMass)./2.0-bottomLinkHeight.*t8.*t11.*topLinkMass-bottomLinkHeight.*t9.*t11.*topLinkMass-t8.*t16.*topLinkRCoM.*topLinkMass-t9.*t16.*topLinkRCoM.*topLinkMass-t10.*t16.*topLinkRCoM.*topLinkMass-boardDTheta.*bottomLinkRCoM.*bottomLinkMass.*bottomLinkDTheta.*t11.*2.0-boardDTheta.*bottomLinkDTheta.*bottomLinkHeight.*t11.*topLinkMass.*2.0-boardDTheta.*bottomLinkDTheta.*t16.*topLinkRCoM.*topLinkMass.*2.0-boardDTheta.*t16.*topLinkRCoM.*topLinkMass.*topLinkDTheta.*2.0-bottomLinkDTheta.*t16.*topLinkRCoM.*topLinkMass.*topLinkDTheta.*2.0;t23+t24+t25+t26+t27+t28-(boardHeight.*g.*t3.*topLinkMass)./2.0-(boardHeight.*bottomLinkMass.*g.*t3)./2.0-(boardHeight.*bottomLinkRCoM.*bottomLinkMass.*t4.*t9)./2.0-(boardHeight.*bottomLinkHeight.*t4.*t9.*topLinkMass)./2.0-(boardHeight.*t9.*t14.*topLinkRCoM.*topLinkMass)./2.0-(boardHeight.*t10.*t14.*topLinkRCoM.*topLinkMass)./2.0-boardDTheta.*boardHeight.*bottomLinkRCoM.*bottomLinkMass.*bottomLinkDTheta.*t4-boardDTheta.*boardHeight.*bottomLinkDTheta.*bottomLinkHeight.*t4.*topLinkMass-boardDTheta.*boardHeight.*bottomLinkDTheta.*t14.*topLinkRCoM.*topLinkMass-boardDTheta.*boardHeight.*t14.*topLinkRCoM.*topLinkMass.*topLinkDTheta-boardHeight.*bottomLinkDTheta.*t14.*topLinkRCoM.*topLinkMass.*topLinkDTheta;t23+t24+t25+t26+t27+t28+(boardHeight.*bottomLinkRCoM.*bottomLinkMass.*t4.*t8)./2.0+(boardHeight.*bottomLinkHeight.*t4.*t8.*topLinkMass)./2.0+(boardHeight.*t8.*t14.*topLinkRCoM.*topLinkMass)./2.0;(topLinkRCoM.*topLinkMass.*(g.*t15.*-2.0+boardHeight.*t8.*t14+bottomLinkHeight.*t5.*t8.*2.0+bottomLinkHeight.*t5.*t9.*2.0+boardDTheta.*bottomLinkDTheta.*bottomLinkHeight.*t5.*4.0))./2.0];
