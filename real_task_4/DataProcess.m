CKNLDF =  ["BiggerK1NLData--","BiggerK2NLData--"];
LNDF =  ["Data0-","Data0+","Data-0","Data--","Data+0","Data+-","Data-+","Data++"];
NLDF =   ["NLData0-","NLData0+","NLData-0","NLData--","NLData+0","NLData+-","NLData-+","NLData++"];
for i = (1:2)
    figure("Name","ChangedControl")
    grid on;
    data = readmatrix(CKNLDF(i));
    plot(data(:,1),data(:,2));
    print("DataChangedKLinearControl"+i,"-djpeg");
end
close all;
for i = (1:8)
    figure("Name","DataFromLinearControl")
    grid on;
    data = readmatrix(LNDF(i));
    plot(data(:,1),data(:,2));
    print("DataFromLinearControl"+i,"-djpeg");
end
close all;
for i = (1:8)
    figure("Name","DataFromNotLinearControl")
    grid on;
    data = readmatrix(NLDF(i));
    plot(data(:,1),data(:,2));
    print("DataFromNotLinearControl"+i,"-djpeg");
end
close all;