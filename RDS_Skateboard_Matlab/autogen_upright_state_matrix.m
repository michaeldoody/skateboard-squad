function A = autogen_upright_state_matrix(boardI,boardMass,bottomLinkI,bottomLinkRCoM,bottomLinkMass,bottomLinkHeight,g,topLinkI,topLinkRCoM,topLinkMass)
%AUTOGEN_UPRIGHT_STATE_MATRIX
%    A = AUTOGEN_UPRIGHT_STATE_MATRIX(BOARDI,BOARDMASS,BOTTOMLINKI,BOTTOMLINKRCOM,BOTTOMLINKMASS,BOTTOMLINKHEIGHT,G,TOPLINKI,TOPLINKRCOM,TOPLINKMASS)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
<<<<<<< HEAD
%    01-May-2020 09:54:05
=======
%    30-Apr-2020 22:23:38
>>>>>>> 57d8f43bd5fa33b53ad544a97974782202f2f5fc

t2 = boardMass.*topLinkI;
t3 = bottomLinkMass.*topLinkI;
t4 = topLinkI.*topLinkMass;
t5 = bottomLinkRCoM.^2;
t6 = bottomLinkHeight.^2;
t7 = topLinkRCoM.^2;
t8 = boardI.*boardMass.*bottomLinkI;
t9 = boardI.*bottomLinkI.*bottomLinkMass;
t10 = boardI.*bottomLinkI.*topLinkMass;
t21 = boardMass.*bottomLinkHeight.*topLinkRCoM.*topLinkMass;
t22 = bottomLinkRCoM.*bottomLinkMass.*topLinkRCoM.*topLinkMass;
t23 = bottomLinkMass.*bottomLinkHeight.*topLinkRCoM.*topLinkMass;
t26 = boardI.*boardMass.*bottomLinkRCoM.*bottomLinkHeight.*topLinkMass;
t28 = boardMass.*bottomLinkI.*bottomLinkRCoM.*bottomLinkHeight.*topLinkMass;
t30 = boardI.*boardMass.*bottomLinkRCoM.*topLinkRCoM.*topLinkMass;
t33 = boardMass.*bottomLinkI.*bottomLinkRCoM.*topLinkRCoM.*topLinkMass;
t47 = boardI.*bottomLinkRCoM.*bottomLinkMass.*bottomLinkHeight.*topLinkMass.*2.0;
t48 = bottomLinkI.*bottomLinkRCoM.*bottomLinkMass.*bottomLinkHeight.*topLinkMass.*2.0;
t11 = bottomLinkRCoM.*t2;
t12 = bottomLinkHeight.*t2;
t13 = bottomLinkRCoM.*t3;
t14 = bottomLinkHeight.*t3;
t15 = t2.*topLinkRCoM;
t16 = bottomLinkRCoM.*t4;
t17 = t3.*topLinkRCoM;
t18 = bottomLinkHeight.*t4;
t19 = t4.*topLinkRCoM;
t20 = boardI.*bottomLinkI.*t4;
t24 = boardI.*bottomLinkI.*t2;
t25 = boardI.*bottomLinkI.*t3;
t38 = -t8;
t39 = -t9;
t40 = -t10;
t41 = -t21;
t42 = -t23;
t43 = boardI.*boardMass.*bottomLinkMass.*t5;
t44 = boardMass.*bottomLinkI.*bottomLinkMass.*t5;
t45 = boardI.*t3.*t5;
t46 = bottomLinkI.*t3.*t5;
t49 = t7.*t10;
t50 = boardI.*t4.*t6;
t51 = boardMass.*bottomLinkRCoM.*t7.*topLinkMass;
t52 = bottomLinkI.*t4.*t6;
t53 = boardI.*t22.*2.0;
t54 = bottomLinkI.*t22.*2.0;
t55 = boardI.*t4.*t7;
t56 = bottomLinkI.*t4.*t7;
t59 = boardI.*bottomLinkMass.*t5.*topLinkMass.*2.0;
t60 = bottomLinkI.*bottomLinkMass.*t5.*topLinkMass.*2.0;
t61 = bottomLinkRCoM.*bottomLinkMass.*t7.*topLinkMass.*2.0;
t62 = boardI.*bottomLinkMass.*t2.*t5;
t63 = bottomLinkI.*bottomLinkMass.*t2.*t5;
t64 = t7.*t8.*topLinkMass;
t65 = boardI.*t2.*t6.*topLinkMass;
t66 = t7.*t9.*topLinkMass;
t68 = boardI.*t3.*t6.*topLinkMass;
t69 = bottomLinkI.*t2.*t6.*topLinkMass;
t71 = bottomLinkI.*t3.*t6.*topLinkMass;
t74 = boardI.*t2.*t7.*topLinkMass;
t75 = boardI.*t3.*t7.*topLinkMass;
t76 = bottomLinkI.*t2.*t7.*topLinkMass;
t77 = bottomLinkI.*t3.*t7.*topLinkMass;
t27 = boardI.*bottomLinkHeight.*t13;
t29 = bottomLinkI.*bottomLinkHeight.*t13;
t31 = boardI.*t13.*topLinkRCoM;
t32 = boardI.*bottomLinkHeight.*t16;
t34 = bottomLinkI.*t13.*topLinkRCoM;
t35 = bottomLinkI.*bottomLinkHeight.*t16;
t36 = boardI.*t16.*topLinkRCoM;
t37 = bottomLinkI.*t16.*topLinkRCoM;
t57 = boardI.*t18.*topLinkRCoM.*2.0;
t58 = bottomLinkI.*t18.*topLinkRCoM.*2.0;
t67 = t45.*topLinkMass;
t70 = t46.*topLinkMass;
t78 = boardI.*t12.*topLinkRCoM.*topLinkMass.*2.0;
t80 = boardI.*t14.*topLinkRCoM.*topLinkMass.*2.0;
t81 = bottomLinkI.*t12.*topLinkRCoM.*topLinkMass.*2.0;
t83 = bottomLinkI.*t14.*topLinkRCoM.*topLinkMass.*2.0;
t84 = -t43;
t85 = -t44;
t86 = -t59;
t87 = -t60;
t92 = t7.*t59;
t93 = t7.*t60;
t94 = t7.*t43.*topLinkMass;
t95 = t7.*t44.*topLinkMass;
t96 = t2+t3+t4+t22+t41+t42;
t97 = t11+t12+t13+t14+t15+t16+t17+t18+t19+t51+t61;
t72 = t27.*topLinkMass.*2.0;
t73 = t29.*topLinkMass.*2.0;
t79 = t31.*topLinkMass.*2.0;
t82 = t34.*topLinkMass.*2.0;
t98 = t26+t28+t30+t33+t38+t39+t40+t47+t48+t53+t54+t84+t85+t86+t87;
t99 = t27+t29+t31+t32+t34+t35+t36+t37+t45+t46+t49+t50+t52+t55+t56+t57+t58+t92+t93;
t88 = -t72;
t89 = -t73;
t90 = -t79;
t91 = -t82;
t100 = t20+t24+t25+t62+t63+t64+t65+t66+t67+t68+t69+t70+t71+t74+t75+t76+t77+t78+t80+t81+t83+t88+t89+t90+t91+t94+t95;
t101 = 1.0./t100;
t102 = boardI.*bottomLinkMass.*g.*t97.*t101;
t103 = bottomLinkI.*bottomLinkMass.*g.*t97.*t101;
t104 = bottomLinkMass.*g.*t98.*t101.*topLinkRCoM;
t106 = bottomLinkMass.*g.*t99.*t101;
t105 = -t104;
A = reshape([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t106,0.0,t103,t102,t105,0.0,0.0,0.0,0.0,0.0,t106,0.0,t103,t102,t105,0.0,0.0,0.0,0.0,0.0,bottomLinkMass.*g.*t101.*topLinkRCoM.*(boardI.*t13+boardI.*t18+boardI.*t19+bottomLinkI.*t13+bottomLinkI.*t18+bottomLinkI.*t19+t10.*topLinkRCoM-boardI.*bottomLinkHeight.*t22-bottomLinkI.*bottomLinkHeight.*t22+boardI.*bottomLinkMass.*t5.*topLinkRCoM.*topLinkMass+bottomLinkI.*bottomLinkMass.*t5.*topLinkRCoM.*topLinkMass),0.0,bottomLinkI.*bottomLinkMass.*g.*t96.*t101.*topLinkRCoM,boardI.*bottomLinkMass.*g.*t96.*t101.*topLinkRCoM,bottomLinkMass.*g.*t101.*topLinkRCoM.*(t8+t9+t10+t43+t44-t47-t48+boardI.*t21-boardI.*t22+boardI.*t23+bottomLinkI.*t21-bottomLinkI.*t22+bottomLinkI.*t23+boardI.*boardMass.*t6.*topLinkMass+boardI.*bottomLinkMass.*t5.*topLinkMass+boardI.*bottomLinkMass.*t6.*topLinkMass+boardMass.*bottomLinkI.*t6.*topLinkMass+bottomLinkI.*bottomLinkMass.*t5.*topLinkMass+bottomLinkI.*bottomLinkMass.*t6.*topLinkMass),1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0],[10,10]);
