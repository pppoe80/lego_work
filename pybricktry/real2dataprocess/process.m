kdata = readmatrix("k_data");
k = 2.0;
Tm = 0.6;
for i = 1:6
    data = readmatrix("Data"+i);
    target = data(1,2);
    ki = kdata(i,1);
    kd = kdata(i,2);
    kp = kdata(i,3);
    if i == 2 || i >=3
        stedyerr(i) = data(1,2) - data(end,end);
    end
    figure('Name','angle-t_graph');
    grid on;
    grid minor;
    plot(data(3:end,1)-data(3,1),data(3:end,2));
    maxangle = max(data(3:end,2));
    if i == 1 || i >=3
        oversangle(i) = maxangle - data(1,2); 
    end
    ylim([0,maxangle+30]);
    hold on;
    xlabel('time,millseconds');
    ylabel('angle,degrees');
    si = sim('call_model');
    plot(si.yout{1}.Values,'r');
    print('compare'+"pic_data"+string(i),'-djpeg');
end
close all;