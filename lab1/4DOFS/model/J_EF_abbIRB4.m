function Jef_0=J_EF_abbIRB4(q,L)
%J_EF_ABBIRB4 calculates the geometric Jacobian of the end-effector wrt the
%robot base (link 0)
% input:
%   q: is the joint position vector 1X4
%   L: is the kinematic parameter array (see abbIRB4_params.m)
% return:
%   Jef_0: End-effector Jacobian wrt robot base (6Xn)


% Joint Positions
q1=q(1);
q2=q(2);
q3=q(3);
q4=q(4);

% Kinematic paramters
L1=L(1);
L2=L(2);
L7=L(3);
L8=L(4);
al=L(5);

% Common substitutions
c1=cos(q1);
s1=sin(q1);

c2=cos(q2);
s2=sin(q2);

c234=cos(q2 + q3 + q4);
s234=sin(q2 + q3 + q4);

ca23=cos(al + q2 + q3);
sa23=sin(al + q2 + q3);

% TODO: Calculate the Jacobian of the end-effector wrt link 0 (6xn)
[HT_0, ~] = getAbsoluteHT_abbIRB4(q,L, eye(4));
    T0_0 = HT_0{1};
    T1_0 = HT_0{2};
    T2_0 = HT_0{3};
    T3_0 = HT_0{4};
    T4_0 = HT_0{5};

    J1_0_contribution = [cross(T0_0(1:3, 3), (T4_0(1:3, 4) - T0_0(1:3, 4))); T0_0(1:3, 3)];
    J2_0_contribution = [cross(T1_0(1:3, 3), (T4_0(1:3, 4) - T1_0(1:3, 4))); T1_0(1:3, 3)];
    J3_0_contribution = [cross(T2_0(1:3, 3), (T4_0(1:3, 4) - T2_0(1:3, 4))); T2_0(1:3, 3)];
    J4_0_contribution = [cross(T3_0(1:3, 3), (T4_0(1:3, 4) - T3_0(1:3, 4))); T3_0(1:3, 3)];
    % Calculate the Jacobian of the end-effector wrt link 0 (6xn)
    Jef_0 = [J1_0_contribution, J2_0_contribution, J3_0_contribution, J4_0_contribution];
end