
%% 
% Description: Application entry point.
% 
% Inputs: none
% 
% Outputs: none
% 
% Notes:
%% Initialize environment
function main 
clear;
close all;
clc;

init_env();
%% Initialize parameters

params = initial_params;
global i
%% Visualize the robot in its initial state

% first five elements are configs (boardX boardY boardTheta bottomLinkTheta topLinkTheta
% last five are velocities corresponding

boardX_init = 0;
boardY_init = 0;
boardTheta_init = 0;
bottomLinkTheta_init = 0;
topLinkTheta_init = 0;
boardDX_init = 0;
boardDY_init = 0;
boardDTheta_init = 10;
bottomLinkDTheta_init = 0;
topLinkDTheta_init = 0;


x_IC = [boardX_init; boardY_init; boardTheta_init;...
        bottomLinkTheta_init; topLinkTheta_init; ...
        boardDX_init; boardDY_init; boardDTheta_init;...
        bottomLinkDTheta_init; topLinkDTheta_init];
    
i = 0;
 
plot_robot(x_IC,params,'new_fig',false);

%% main.m
%
% Description:
%   Application entry point.
%
% Inputs: none
%
% Outputs: none
%
% Notes:



%% Initialize environment

init_env();

%% Initialize parameters
params = initial_params;

%% Set up events using odeset
options = odeset('Events',@robot_events);

%% Simulate the robot forward in time     

% initial conditions
tnow = 0.0;            
% starting time

% start with null matrices for holding results -- we'll be adding to these
% with each segment of the simulation
tsim = [];
xsim = [];
F_list = [];
DX_Matrix = [];
DY_Matrix = [];
DTheta_Matrix = [];
DTheta_bottomlink_Matrix = [];
DTheta_toplink_Matrix = [];
TotEnergy = [];
T = [];


% create a place for constraint forces
F = [];

while tnow < params.sim.tfinal

    tspan = [tnow params.sim.tfinal];
    [tseg, xseg, ~, ~, ie] = ode45(@robot_dynamics, tspan, x_IC, options);

    % augment tsim and xsim; renew ICs
    tsim = [tsim;tseg];
    xsim = [xsim;xseg];
    tnow = tsim(end);
    x_IC = xsim(end,:)
    
    % compute the constraint forces that were active during the jump
    [Fseg] = constraint_forces(tseg,xseg',params);
     F_list = [F_list,Fseg];
    
    % if simulation terminated before tfinal, determine which constaints
    % are still active, then continue integration
    if tseg(end) < params.sim.tfinal  % termination was triggered by an event
        switch params.sim.constraints
            case ['true','false']  % the left foot was on the ground prior to termination
                 params.sim.constraints = ['false','false'];  % now the left foot is off
            case ['false','true'] % the right foot only was on the ground prior to termination
                 params.sim.constraints = ['false','false'];  % now the right foot is off
            case ['true','true'] % both feet were on the ground prior to termination
                 if ie == 1
                    params.sim.constraints = ['false','true'];  % now the left foot is off
                 else
                    params.sim.constraints = ['true','false'];  % now the right foot is off
                 end
        end
    end
end

%%  Plot Results %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Begin with plot of ground reaction versus weight, to be sure we're
% pushing off and then leaving the ground
figure;
subplot(2,3,1)
plot(T, DX_Matrix, 'r-', 'LineWidth', 2);
ylabel('DX (m/s)');
xlabel('time (sec)');
subplot(2,3,2)
plot(T, DY_Matrix, 'r-', 'LineWidth', 2);
ylabel('DY (m/s)');
xlabel('time (sec)');
subplot(2,3,3)
plot(T, DTheta_Matrix, 'r-', 'LineWidth', 2);
ylabel('DTheta (rad/s)');
xlabel('time (sec)');
subplot(2,3,4)
plot(T, DTheta_bottomlink_Matrix, 'r-', 'LineWidth', 2);
ylabel('DTheta bottomLink (rad/s)');
xlabel('time (sec)');
subplot(2,3,5)
plot(T, DTheta_toplink_Matrix, 'r-', 'LineWidth', 2);
ylabel('DTheta topLink (rad/s)');
xlabel('time (sec)');
subplot(2,3,6)
plot(T, TotEnergy, 'b-', 'LineWidth', 2);
ylabel('Energy (J)');
xlabel('time (sec)');
hold off



% Now let's animate

% Let's resample the simulator output so we can animate with evenly-spaced
% points in (time,state).
% 1) deal with possible duplicate times in tsim:
% (https://www.mathworks.com/matlabcentral/answers/321603-how-do-i-interpolate-1d-data-if-i-do-not-have-unique-values
tsim = cumsum(ones(size(tsim)))*eps + tsim;

% 2) resample the duplicate-free time vector:
t_anim = 0:params.sim.dt:tsim(end);

% 3) resample the state-vs-time array:
x_anim = interp1(tsim,xsim,t_anim);
x_anim = x_anim'; % transpose so that xsim is 10xN (N = number of timesteps)

% 4) resample the constraint forces-vs-time array:
F_anim = interp1(tsim,F_list',t_anim);
F_anim = F_anim';

animate_robot(x_anim(1:5,:), F_anim, params,'trace_board_com',true,...
    'trace_bottomLink_com',true,'trace_topLink_com',true,'trace_robot_com',...
     true,'show_constraint_forces',true,'video',true);
fprintf('Done!\n');

%% BELOW HERE ARE THE NESTED FUNCTIONS, ROBOT_DYNAMICS AND ROBOT_EVENTS

%% THEY HAVE ACCESS TO ALL VARIABLES IN MAIN

%% robot_dynamics.m %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Description:
%   Computes the constraint forces: 
%       Fnow = inv(A*Minv*A')*(A*Minv*(Q-H) + Adotqdot)
%
%   Also computes the derivative of the state:
%       x_dot(1:5) = (I - A'*inv(A*A')*A)*x(6:10)
%       x_dot(6:10) = inv(M)*(Q - H - A'F)
%
% Inputs:
%   t: time (scalar)
%   x: the 10x1 state vector
%   params: a struct with many elements, generated by calling init_params.m
%
% Outputs:
%   dx: derivative of state x with respect to time.
%   energy: total energy of the system at state x

function [dx] = robot_dynamics(t,x)

% for convenience, define q_dot
dx = zeros(numel(x),1);
nq = numel(x)/2;    % assume that x = [q;q_dot];
q_dot = x(nq+1:2*nq);

% solve for control inputs at this instant

bottomMotorTorque = interp1(params.bottomMotor.time,params.bottomMotor.torque,t);
topMotorTorque = interp1(params.topMotor.time,params.topMotor.torque,t);
Q = [0;0;0;bottomMotorTorque;topMotorTorque];

% find the parts that don't depend on constraint forces
H = H_eom(x,params);
Minv = inv_mass_matrix(x,params);
[A_all,Hessian] = constraint_derivatives(x,params);

% compute energy

TE_now = totalEnergy(x, params);
DX_Matrix = [DX_Matrix; x(6)]; % matrix keeping track of DX for skateboard
DY_Matrix = [DY_Matrix; x(7)]; % matrix keeping track of DY for skateboard
DTheta_Matrix = [DTheta_Matrix; x(8)]; % matrix keeping track of DTheta for skateboard
DTheta_bottomlink_Matrix = [DTheta_bottomlink_Matrix; x(9)];
DTheta_toplink_Matrix = [DTheta_toplink_Matrix; x(9)];
TotEnergy = [TotEnergy; TE_now]; % matrix keeping track of total energy
T = [T; t]; % matrix keeping track of time

% build the constraints, forces, and solve for acceleration 
switch params.sim.constraints  
    case ['false','false']     % both wheels are off the ground
        dx(1:nq) = q_dot;
        dx(nq+1:2*nq) = Minv*(Q - H);
        F = [0;0];
    case ['true','false']      % left wheel is on the ground and right is off
        A = A_all(1,:);
        Adotqdot = [q_dot'*Hessian(:,:,1)*q_dot];
        Fnow = (A*Minv*A')\(A*Minv*(Q - H) + Adotqdot);
        dx(1:nq) = (eye(nq) - A'*((A*A')\A))*x(6:10);
        dx(nq+1:2*nq) = Minv*(Q - H - A'*Fnow);
        F = [Fnow;0];
    case ['false','true']      % right wheel is on the ground and left is off
        A = A_all(2,:);
        Adotqdot = [q_dot'*Hessian(:,:,2)*q_dot];
        Fnow = (A*Minv*A')\(A*Minv*(Q - H) + Adotqdot);
        dx(1:nq) = (eye(nq) - A'*((A*A')\A))*x(6:10);
        dx(nq+1:2*nq) = Minv*(Q - H - A'*Fnow);
        F = [0;Fnow];
    case ['true','true']      % both wheels are on the ground
        A = A_all([1,2],:);
        Adotqdot = [q_dot'*Hessian(:,:,1)*q_dot;
                    q_dot'*Hessian(:,:,2)*q_dot];
        Fnow = (A*Minv*A')\(A*Minv*(Q - H) + Adotqdot);
        dx(1:nq) = (eye(nq) - A'*((A*A')\A))*x(6:10);
        dx(nq+1:2*nq) = Minv*(Q - H - A'*Fnow);
        F = [Fnow(1);Fnow(2)];
end

end
%% end of robot_dynamics.m


%% Event function for ODE45 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description:
%   Event function that is called when a constraint becomes inactive (or, in the future, active) 
%
% Inputs:
%   t and x are required, but not used
%   F is shared with parent function
%
% Outputs:
%   value
%   isterminal
%   direction
function [value,isterminal,direction] = robot_events(~,~)

    switch params.sim.constraints  
        case ['false','false']      % both feet are off the ground
            value = 1;
            isterminal = 0;
            direction = 0;
        case ['true','false']      % left foot is on the ground and right is off
            value = F(1);
            isterminal = 1;
            direction = -1;
        case ['false','true']      % right foot is on the ground and left is off
            value = F(2);
            isterminal = 1;
            direction = -1;
        case ['true','true']      % both feet are on the ground
            value = [F(1);F(2)];
            isterminal = ones(2,1);
            direction = -ones(2,1);
    end


end
%% end of robot_events.m 
%% End of main.m
end






% 
% 
% tspan_passive = 0:params.sim.dt:5;
% [tsim_passive, xsim_passive] = ode45(@(t,x) robot_dynamics(...
%     t,x,[0;0],params,'controller','passive'),...
%     tspan_passive, x_IC');
% 
% size(xsim_passive)
% 
% % tranpose xsim_passive so that it is 4xN (N = number of timesteps):
% xsim_passive = xsim_passive'; % required by animate_robot.m
% 
% figure;
% subplot(2,1,1), plot(tsim_passive,xsim_passive(1,:),'b-',...
%                      tsim_passive,xsim_passive(2,:),'r-','LineWidth',2);
% subplot(2,1,2), plot(tsim_passive,xsim_passive(3,:),'b:',...
%                      tsim_passive,xsim_passive(4,:),'r:','LineWidth',2);
% 
% 
% pause(1); % helps prevent animation from showing up on the wrong figure
% animate_robot(xsim_passive(1:5,:),params,'trace_board_com',true,...
%     'trace_bottomLink_com',true,'trace_topLink_com',true,...
%     'trace_robot_com',true,'video',true);
% fprintf('Done passive simulation.\n');

% %% Control the unstable equilibrium with LQR
% A = upright_state_matrix(params);
% B = upright_input_matrix(params);
% 
% % numerical verify the rank of the controllability matrix:
% Co = [B, A*B, (A^2)*B, (A^3)*B];
% fprintf('rank(Co) = %d.\n',rank(Co));
% 
% % control design: weights Q and R:
% Q = diag([5000,100,1,1]);    % weight on regulation error
% R = 1;                  % weight on control effort
% 
% % compute and display optimal feedback gain matrix K:
% K = lqr(A,B,Q,R);
% buf = '';
% for i = 1:size(K,2)
%     buf = [buf,'%5.3f '];
% end
% buf = [buf,'\n'];
% fprintf('LQR: K = \n');
% fprintf(buf,K');
% 
% % we could ask what are the eigenvalues of the closed-loop system:
% eig(A - B*K)
% 
% % add K to our struct "params":
% params.control.inverted.K = K;
% 
% % Simulate the robot under this controller:
% tspan_stabilize = 0:params.sim.dt:5;
% [tsim_stabilize, xsim_stabilize] = ode45(@(t,x) robot_dynamics(...
%     t,x,0,params,'controller','stabilize'),...
%     tspan_stabilize, x_IC');
% 
% % tranpose xsim_passive so that it is 4xN (N = number of timesteps):
% xsim_stabilize = xsim_stabilize'; % required by animate_robot.m
% 
% figure;
% subplot(2,1,1), plot(tsim_stabilize,xsim_stabilize(1,:),'b-',...
%                      tsim_stabilize,xsim_stabilize(2,:),'r-','LineWidth',2);
% subplot(2,1,2), plot(tsim_stabilize,xsim_stabilize(3,:),'b:',...
%                      tsim_stabilize,xsim_stabilize(4,:),'r:','LineWidth',2);
% pause(1); % helps prevent animation from showing up on the wrong figure
% 
% 
% animate_robot(xsim_stabilize(1:2,:),params,'trace_cart_com',true,...
%     'trace_pend_com',true,'trace_pend_tip',true,'video',true);
% fprintf('Done passive simulation.\n');