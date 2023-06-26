% 通过将simulink中积分器等模块初始值设为变量
% 可以使用该文件修改对应空间中的变量实现初始值的变化
% 可自行查阅assignin()以及sim()函数的用法

% 注意由于matlab版本的差异
% simulink输出到工作工具的数据可能并不需要一个名为out的结构体
% 请自行修改

clear;
clc;
close all;

for i = -2:0.2:2
    if i <= 0
        assignin('base','temp_var_2',i-0.01);
    else
        assignin('base','temp_var_2',i+0.01);
    end
    assignin('base','temp_var_1',-2*i);
    out=sim('temp_model');
    hold on;
    plot(out.c, out.dc);
end

for i = -2:0.2:2
    assignin('base','temp_var_1',i);
    if i <= 0
        assignin('base','temp_var_2',i/(-2)-0.01);
    else
        assignin('base','temp_var_2',i/(-2)+0.01);
    end
    out=sim('temp_model');
    hold on;
    plot(out.c, out.dc);
end

refline(3,0);
refline(-2,0);

% 限定显示范围
xlim([-10,10]);
ylim([-10,10]);
