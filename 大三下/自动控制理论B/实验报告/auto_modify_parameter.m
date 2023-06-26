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