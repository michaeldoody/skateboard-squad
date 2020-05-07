%% constraint_forces.m
%
% Description:
%   Output function to compute the constraint forces acting on the jumping
%   robot
%
% Inputs:
%   tseg: the array of time values selected by ode45
%   xseg: the 10xlength(tseg) array of states computed by ode45;
%   params: a struct with many elements, generated by calling init_params.m
%
% Outputs:
%   Fseg: a 2xlength(tsim) array of constraint forces

function [Fseg] = constraint_forces(tseg,xseg,params)

Fseg = zeros(2,length(tseg));

for i=1:length(tseg)
    t = tseg(i);
    x = xseg(:,i);
    dq = x(6:10);
    
    % for convenience, define q and q_dot
    nq = numel(x)/2;    % assume that x = [q;q_dot];
    q = x(1:nq);
    dq = x(nq+1:2*nq);

    % solve for control inputs at this instant
    bottomLinkTau = interp1(params.bottomMotor.time,params.bottomMotor.torque,t);
    topLinkTau = interp1(params.topMotor.time,params.topMotor.torque,t);
    Q = [0;0;0;bottomLinkTau;topLinkTau];

    % find the parts that don't depend on constraint forces
    H = H_eom(x,params);
    M = mass_matrix(x,params);
    Minv = inv_mass_matrix(x,params);
    [A_all,Hessian] = constraint_derivatives(x,params);

    % build the constraints and forces 
    switch params.sim.constraints  
        case ['false','false']      % both wheels are off the ground
            Fseg(:,i) = zeros(2,1);
            
        case ['true','false']      % left wheel is on the ground and right is off
            A = A_all(1,:);
            Adotqdot = [dq'*Hessian(:,:,1)*dq];
            F = inv(A*Minv*A')*(A*Minv*(Q - H) + Adotqdot);
            Fseg(:,i) = [F(1); 0];
            
        case ['false','true']      % right wheel is on the ground and left is off
            A = A_all(2,:);
            Adotqdot = [dq'*Hessian(:,:,2)*dq];
            F = inv(A*Minv*A')*(A*Minv*(Q - H) + Adotqdot);
            Fseg(:,i) = [0; F(1)];
            
        case ['true','true']      % both wheels are on the ground
            A = A_all([1,2],:);
            Adotqdot = [dq'*Hessian(:,:,1)*dq;
                        dq'*Hessian(:,:,2)*dq];
            F = inv(A*Minv*A')*(A*Minv*(Q - H) + Adotqdot);
            Fseg(:,i) = [F(1); F(2)];
    end
end

end

