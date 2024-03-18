% This script defines a continuous-time nonlinear Monocopter model and
% generates a state function and its Jacobian function used by the
% nonlinear MPC controller in the Monocopter trajectory tracking.

% can compare with LaneFollowingStateFcn(), inputs include disturbance rejection
% openExample('mpc/LaneFollowingUsingNMPCExample') *************** Mon

% Create symbolic functions for time-dependent angles of wrt world frame
% rpy
% phi: roll angle = body roll/disk roll to offset centrifugal force induced   
% theta: pitch angle  = body roll/disk pitch
% psi: disk yaw angle = desired disk heading 

syms phi(t) theta(t) psi(t)

%% Transformation matrix for angular velocities from inertial frame to body frame

W = [ 1,  0,        -sin(theta);
      0,  cos(phi),  cos(theta)*sin(phi);
      0, -sin(phi),  cos(theta)*cos(phi) ];

% Rotation matrix R_ZYX from body frame to inertial frame 
R = rotationMatrixEulerZYX(phi,theta,psi);

%% Create symbolic variables for diagonal elements of inertia matrix
syms Ixx Iyy Izz

% Jacobian that relates body frame to inertial frame velocities
I = [Ixx, 0, 0; 0, Iyy, 0; 0, 0, Izz];
J = W.'*I*W; % without the inertia frame velocities appended yet; (W transpose) mtimes I mtimes W
% J = cross(W,I*W); %% value always amount to matrix full of null values

% for euler lagrange component
%dJ_dt = diff(J);
%C = [diff(phi,t), diff(theta,t), diff(psi,t)]*J; % WD.'*I*WD
%h_dot_J = [diff(phi,t), diff(theta,t), diff(psi,t)]*J;
%grad_temp_h = transpose(jacobian(h_dot_J,[phi theta psi]));
%C = dJ_dt - 1/2*grad_temp_h;
%C = subsStateVars(C,t);

% Define fixed parameters and control inputs
% Cl: lift coefficient  
% dl: linear drag term
% dq: quadratic drag term
% m: monocopter's mass
% g: gravity
% r: monocopter length 
% rho: air density
% chord: chord length
% ui: squared angular velocity of rotor i as control input
syms Cl dl dq m g r rho chord u1 u2 u3 

% Disk torque in the direction of phi, theta, psi, still comtemplating
% about adding the disturbance for phi (u3) - roll, pitch, yaw
tau_disk_rate = [u3; u2; 0];

% Disk/body collective thrust
T = (Cl*rho*chord*(u1.^2)*(r.^3))/6; % input the thrust eqn here

% Create symbolic functions for time-dependent positions, this cant be
% altered in real-time when MPC is on loop
syms x(t) y(t) z(t)

% Create symbolic functions for DRAG
syms Dx Dy Dz

% Create state variables for the disk consisting of positions, angles,
% and their derivatives - bod roll = disk pitch
state = [x; y; z; phi; theta; psi; diff(x,t); diff(y,t); ...
    diff(z,t); diff(phi,t); diff(theta,t); diff(psi,t)]; % INPUT STATE in here, [phi,theta,psi] is roll p
state = subsStateVars(state,t);
% state variables above are all in world/inertia frame

f = [ % Set time-derivative of the positions and angles, this is wat the bottom 2 eqns equate to, then after which i think they prob use RK4 to integrate *****
      state(7:12);

      % Equations for linear accelerations of the center of mass including
      % inclusions for linear drag
      % -g*[0;0;1] + R*[0;0;T]/m;
      -g*[0;0;1] - [0;0;sqrt(((Dx*state(7)).^2)+((Dy*state(8)).^2)+((Dz*state(7)).^2))] + [0;0;T]/m;

      % Euler–Lagrange equations for angular dynamics, needa include the
      % disturbance here...working on this part, tau_disk_rate is the solution
      % inv(J)*(tau_disk_rate - C*state(10:12)) 
      inv(I)*(tau_disk_rate - J*[state(10);state(11);0;]) - [dl*state(10) + dq*(state(10)^2);dl*state(11) + dq*(state(11)^2);0;]  % if the direction of precession is positive or negative, negative or positve centrifugal force compensates
];

f = subsStateVars(f,t);

% Replace fixed parameters with given values here Cl dl dq m g r rho chord u1 u2 u3 
IxxVal = 1.2;
IyyVal = 1.2;
IzzVal = 2.3;
ClVal = 6; % temporal value
dlVal = 0.002;
dqVal = 0.005;
mVal = 0.241; % mass
gVal = 9.81; % gravity
rVal = 0.38; % wingspan
rhoVal = 1.225; % air density
chordVal = 0.24/rVal; % wingarea / wingspan
DxVal = 0;
DyVal = 0;
DzVal = 0;

f = subs(f, [Ixx Iyy Izz Cl dl dq m g r rho chord Dx Dy Dz], ...
    [IxxVal IyyVal IzzVal ClVal dlVal dqVal mVal gVal rVal rhoVal chordVal DxVal DyVal DzVal]);
f = simplify(f);

% Calculate Jacobians for nonlinear prediction model
A = jacobian(f,state);
control = [u1; u2; u3];
B = jacobian(f,control);

% Create MonocopterStateFcn.m with current state and control
% vectors as inputs and the state time-derivative as outputs
matlabFunction(f,"File","MonocopterStateFcn", ...
    "Vars",{state,control});

% Create MonocopterStateJacobianFcn.m with current state and control
% vectors as inputs and the Jacobians of the state time-derivative
% as outputs
matlabFunction(A,B,"File","MonocopterStateJacobianFcn", ...
    "Vars",{state,control});

% Confirm the functions are generated successfully
while isempty(which('MonocopterStateJacobianFcn'))
    pause(0.1);
    disp("ready to be tested...");
end


function [Rz,Ry,Rx] = rotationMatrixEulerZYX(phi,theta,psi)
% Euler ZYX angles convention
    Rx = [ 1,           0,          0;
           0,           cos(phi),  -sin(phi);
           0,           sin(phi),   cos(phi) ];
    Ry = [ cos(theta),  0,          sin(theta);
           0,           1,          0;
          -sin(theta),  0,          cos(theta) ];
    Rz = [cos(psi),    -sin(psi),   0;
          sin(psi),     cos(psi),   0;
          0,            0,          1 ];
    if nargout == 3
        % Return rotation matrix per axes
        return;
    end
    % Return rotation matrix from body frame to inertial frame
    Rz = Rz*Ry*Rx;
end

function stateExpr = subsStateVars(timeExpr,var)
    if nargin == 1 
        var = sym("t");
    end
    repDiff = @(ex) subsStateVarsDiff(ex,var);
    stateExpr = mapSymType(timeExpr,"diff",repDiff);
    repFun = @(ex) subsStateVarsFun(ex,var);
    stateExpr = mapSymType(stateExpr,"symfunOf",var,repFun);
    stateExpr = formula(stateExpr);
end

function newVar = subsStateVarsFun(funExpr,var)
    name = symFunType(funExpr);
    name = replace(name,"_Var","");
    stateVar = "_" + char(var);
    newVar = sym(name + stateVar);
end

function newVar = subsStateVarsDiff(diffExpr,var)
    if nargin == 1 
      var = sym("t");
    end
    c = children(diffExpr);
    if ~isSymType(c{1},"symfunOf",var)
      % not f(t)
      newVar = diffExpr;
      return;
    end
    if ~any([c{2:end}] == var)
      % not derivative wrt t only
      newVar = diffExpr;
      return;
    end
    name = symFunType(c{1});
    name = replace(name,"_Var","");
    extension = "_" + join(repelem("d",numel(c)-1),"") + "ot";
    stateVar = "_" + char(var);
    newVar = sym(name + extension + stateVar);
end