function x_wheel = autogen_wheel_position(boardX,boardTheta,boardHeight,boardLength)
%AUTOGEN_WHEEL_POSITION
%    X_WHEEL = AUTOGEN_WHEEL_POSITION(BOARDX,BOARDTHETA,BOARDHEIGHT,BOARDLENGTH)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    02-Jun-2020 04:02:17

x_wheel = boardX-(boardLength.*cos(boardTheta))./2.0+(boardHeight.*sin(boardTheta))./2.0;
