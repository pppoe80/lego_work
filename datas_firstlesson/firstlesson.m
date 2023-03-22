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
    
    figure('name','real_w and approx_w');
    plot(t,w,'Marker','+');
    xlabel('time,sec');
    ylabel('speed,rad/sec');
    grid on;
    hold on;
    grid minor;
    
    w_approx = k*U-k*U*exp(-t/Tm);
    plot(t,w_approx,'r');

    therta_function = @(P,t) P(1)*U*t+P(1)*P(2)*U*exp(-t/P(2));

    P_guess = [1,0.5];
    P_approx = lsqcurvefit(therta_function,P_guess,t,therta);
    k = P_approx(1);
    Tm = P_approx(2);

    figure('name','real_th and approx_th');
    plot(t,therta,'Marker','+');
    xlabel('time,sec');
    ylabel('angle,rad');
    grid on;
    hold on;
    grid minor;
    therta_approx = k*U*t+k*Tm*U*exp(-t/Tm);
    plot(t,therta_approx,'r');

    open_system("call_model.slx");
    load_system("call_model.slx");

    
    simOut = sim("call_model.slx");
    figure('name','sim');
    hold on;
    grid on;
    grid minor;
    xlabel('time,sec');
    ylabel('angle or speed');
    plot(simOut.get('yout'));

end