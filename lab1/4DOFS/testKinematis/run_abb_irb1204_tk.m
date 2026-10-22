% Note: The first time you run this script, it may take some time since it
% has to load the simulink environment (usually a couple of seconds)
% run_abb_irb1204_tk.m

clear; clc; close all;

addpath(genpath(pwd)); 


% 2. 加载机器人模型 (需要 Robotics System Toolbox)
try
    robot = loadrobot('abbIrb120', 'DataFormat', 'column');
catch
    error('无法加载机器人模型，请检查是否安装了 Robotics System Toolbox');
end

% 3. 生成测试数据 (stateData)
% 我们生成 5 组随机姿态来测试
num_frames = 6;
stateData = zeros(num_frames, 6); % IRB120 有 6 个关节
stateData(1, :) = zeros(1, 6);
for i = 2:num_frames
    % 生成 -pi/2 到 pi/2 之间的随机角度
    stateData(i, :) = (rand(1, 6) - 0.5) * pi; 
end

% 4. 设置可视化选项
robotVisualization = 'on'; % 'on' 显示实体模型, 'off' 只显示骨架(如果有冲突)
% 通常建议设为 'off' 以便更清楚地看到你画的坐标系，或者设为 'on' 看整体效果

% 5. 调用你的 plotRobot 函数
fprintf('开始运行验证... 请点击绘图窗口来切换下一帧。\n');
plotRobot_tk(robot, stateData, robotVisualization);