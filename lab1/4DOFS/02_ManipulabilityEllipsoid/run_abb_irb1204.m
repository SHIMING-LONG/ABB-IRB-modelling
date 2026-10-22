% Note: The first time you run this script, it may take some time since it
% has to load the simulink environment (usually a couple of seconds).

% Adds the additional folders with functions into the library path
addpath("C:\Users\little_long\Desktop\ssy156\Lab1\Template\RobotABB_IRB120_4DOFS\DrawCFs"')
% 推荐的修改方式：自动获取当前文件夹并添加所有子文件夹
addpath(genpath(pwd));

% Defines the robot variables needed for the simulation
robot_name="abbIrb120"; 
n_dofs=4;
al=1.343030361054137;


% Opens the simulator. Press play to run the simulation
open('KSimulator_abbIRB4');

