function [loop,X1,Y1,Z1] = gradient(auto, Xinit, Yinit)
    % Algorithm constants
    loop    = 0;
    maxloop = 10000;
    pas     = sqrt(2^-52);
    eps     = 10^-12;

    % Rosenbrock function calculations
    x = -1.5:0.01:1.5;
    y = -1.5:0.01:1.5;
    z = zeros(length(y),length(x));

    for i=1:length(x)
       for j=1:length(y)
           z(j,i)=(1-x(i))^2+100*(y(j)-x(i)^2)^2;
       end
    end

    % Rosenbrock function display
    t  = 0.1:0.2:3;
    t2 = 10.^t;

    figure