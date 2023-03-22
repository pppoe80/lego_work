global k_from_w Tm_from_w;
global k_from_th Tm_from_th;
for n = 1:8
    data = readmatrix("data"+n+".txt");
    U = data(1,1);
    t = data(:,2);
    w = data(:,3);
    therta = data(:,4);

    w = w*pi/180;
    therta = therta*pi/180;

    w_function = @(P,t) P(1)*U-P(1)*U*exp(-t/P(2));

    P_guess = [1,1];
    P_approx = lsqcurvefit(w_function,P_guess,t,w);
    k = P_approx(1);
    Tm = P_approx(2);

    k_from_w(n) = k;
    Tm_from_w(n) = Tm;
    
    figure('name','real_w and approx_w');
    plot(t,w,'Marker','+');
    xlabel('time,sec');
    ylabel('speed,rad/sec');
    grid on;
    hold on;
    grid minor;
    
    w_approx = k*U-k*U*exp(-t/Tm);
    plot(t,w_approx,'r');
    print('real_w and approx_w'+string(n),'-djpeg');

    simOut = sim("call_model.slx");
    figure('name','sim_from_w');
    hold on;
    grid on;
    grid minor;
    xlabel('time,sec');
    ylabel('angle or speed,red is speed');
    plot(simOut.yout{1}.Values);
    plot(simOut.yout{2}.Values,'r');
    print('sim_w_from_w and sim_th_from_w'+string(n),'-djpeg');

    therta_function = @(P,t) P(1)*U*t+P(1)*P(2)*U*exp(-t/P(2));

    P_guess = [1,0.5];
    P_approx = lsqcurvefit(therta_function,P_guess,t,therta);
    k = P_approx(1);
    Tm = P_approx(2);

    k_from_th(n) = k;
    Tm_from_th(n) = Tm;

    figure('name','real_th and approx_th');
    plot(t,therta,'Marker','+');
    xlabel('time,sec');
    ylabel('angle,rad');
    grid on;
    hold on;
    grid minor;
    therta_approx = k*U*t+k*Tm*U*exp(-t/Tm);
    plot(t,therta_approx,'r');
    print('real_th and approx_th'+string(n),'-djpeg');

    open_system("call_model.slx");
    load_system("call_model.slx");

    
    simOut = sim("call_model.slx");
    figure('name','sim_from_th');
    hold on;
    grid on;
    grid minor;
    xlabel('time,sec');
    ylabel('angle or speed,red is speed');
    plot(simOut.yout{1}.Values);
    plot(simOut.yout{2}.Values,'r');
    print('sim_w_from_th and sim_th_from_th'+string(n),'-djpeg');

    
    close all;
end

writelines("k_from_w    Tm_from_w   k_from_th   Tm_from_th  ","k_Tm_Data_out.txt");
writematrix([k_from_w.',Tm_from_w.',k_from_th.',Tm_from_th.'],'k_Tm_Data_out',WriteMode='append');




