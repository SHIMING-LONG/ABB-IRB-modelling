function X=FK_abbIRB4(q,L)
% FK_abbIRB4 Calculates the Forward Kinematics of the EF (Linear Positions)

% Use the obtained function from previous asssignments
[HT_0, HT_W] = getAbsoluteHT_abbIRB4(q,L, eye(4));

Tef_0 = HT_0{end};
X = Tef_0(1:3, 4);