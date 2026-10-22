% This script is meant for evaluation purposes only. Students are highly encouraged to work on their local computers and use this tool only to validate their solutions.

% Load test data
load test.mat
% t, current time
% dt, delta time, for derivative
% HT_0, cell array with all the absolute HT (4x4) wrt robot base
% HT_W, cell array with all the absolute HT (4x4) wrt wcf
% jef_0, end-effector Jacobian wrt robot base (6Xn)
% R4_0_old, persistent variable for previous rotation matrix
% Xp4_0, end-effector linear and angular velocitites wrt robot base (6X1)

%% This is a slightly modified version of the manipulability_abb_irb4 function's content
% (the function calls have been replaced for testing independently of other parts).  

%% Kinematics

% Kinematic Parameters L1,L2,L7,L8 and alpha
L = abbIRB4_params; %L=abbIRB4_params from Exercise 1

%% HT robot base (link0) wrt world cf. In this case is I (4x4)
%TODO: Define the HT of the robot base (link 0) w.r.t. wcf (w)
H0_W=eye(4);


%TODO: Extract the Rotation of the robot base wrt wcf
R0_W=H0_W(1:3, 1:3);

%% Get the absolute HTs for all the joints with the new joint position
%[HT_0, HT_W] = getAbsoluteHT_abbIRB4(Q,L, H0_W); Loaded from test.mat

%% Homogeneous transformation of Link4 wrt base frame (link0)
%TODO: Get the HT of the end-effector (ef) wrt base link (0) and wcf (w)
H4_0=HT_0{end};
H4_W=HT_W{end};

% Orientation of Link4 wrt base frame
%TODO: get the orientation of the ef wrt link 0
R4_0=H4_0(1:3, 1:3);

%% Compute Position of Link4 with respect to the wcf
%TODO: Get the position of the ef wrt wcf 
% Replace the two lines below with your solution
x4_W=H4_W(1:3, 4);
x4_W=zeros(3,1); % needs to be set to run script

%% Compute the Linear and Angular velocities of Link4 (6X1)
%Xp4_0=DifFK_abbIRB4(Q, Qp, L); Loaded from test.mat

% Linear Velocity (Link4) wrt base frame (homogeneous form) (4x1)
% This velocity is obtained from the differential kinematics (Jacobian)
%TODO: get the linear velocity of the ef
x4p_0=Xp4_0(1:3);

% Linear Velocity (Link4) wrt world coordinate frame (4x1)
%TODO: map the linear velocity to the wcf (w)
x4p_w=R0_W * x4p_0;

% Angular Velocity (Link4) wrt base frame
% This velocity is obtained from the differential kinematics (Jacobian)
%TODO: get the angular velocity of the ef wrt link 0
w4_0=Xp4_0(4:6);

% Angular Velocity (Link4) wrt world coordinate frame
%TODO: map the angular velocity to the wcf (w)
w4p_w=R0_W * w4_0;

%% Manipulability Analisys (Ellipsoid)
% jef_0=J_EF_abbIRB4(Q, L); Loaded from test.mat

% Only the sub-jacobian for linear velocity
%TODO: Get the jacobian for the ef linear velocities
% Replace the two lines below with your solution
jef_v=jef_0(1:3, :);
jef_v=zeros(3, 4); % needs to be set to run script

% Reduced form of U with dimension of numel(s). Used to compute the axis of
% the Manipulability ellipsoid axis_i=u_i*s_i
[U,S,V] = svd(jef_v,0);

%U is a rotation matrix (verification)
dU=det(U);
nU=norm(U);
IU=U*U';

su=size(U);
ss=size(S);

ru=rank(U,0.01);

% Manipulability Index
%TODO: compute the manipulability index
w=sqrt(det(jef_v * jef_v'));

% Ellipsoid: Principal Axes
% Rotation includes the robot base to world orientation and the orientation of
% the principal axes of the m-ellipsoid
EAxis_W=R0_W*U;

%Scaling (homogeneous version of S)
S(4,4)=1;

% The total homogeneous transformation for the manipulability ellipsoid 
% includes the orientation wrt world and the ef position wrt 
% world as the center of the m-ellipsoid

Heax_W=[EAxis_W x4_W; 0 0 0 1];

%% Derivative of a Rotation Matrix (M.W. Spong, S. Hutchinson, and
% M. Vidyasagar. Robot modeling and control. pp 117)

% Time derivative of R3_0
if (t==0.0)
    R4_0_old=R4_0;
end

%TODO: Calculate the time derivative of R4_0 (use dt)
R4_0p=(R4_0 - R4_0_old) / dt;

R4_0_old=R4_0;

% Skewsymetric matrix (S([wx;wy;wz])=dR/dt*R')
%TODO: Calculate the S matrix to get the ef's angular velocities wrt link 0
Sw_0=R4_0p * R4_0';

% Angular velocity vector of the ef wrt base frame
% This angular velocity is calculated using S
%TODO: Get the angular velocities from S
w4_0=[Sw_0(3,2); Sw_0(1,3); Sw_0(2,1)];

% Angular velocity vector of the EF wrt wcf
%TODO: Map the angular velocities to the wcf
w4_w=R0_W * w4_0;
