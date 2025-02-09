
function [loop,X1,Y1,Z1] = levenberg(auto, Xinit, Yinit)
    % Algorithm constants
    loop    = 0;
    maxloop = 100;
    eps     = 10^-8;
    lambda  = 10^-5;

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
    hold('off');
    newplot; axis([-1.5,1.5,-1.5,1.5],'square');
    hold('on');
    t  = 0.1:0.2:3;
    t2 = 10.^t;
    contour(x,y,z,t2);

    % Graphical initialization
    %[X,Y] = ginput(1);
    X = Xinit; Y = Yinit;
    plot(X,Y,'+r');
    Xmin = zeros(1, maxloop); Xmin(loop+1) = X;
    Ymin = zeros(1, maxloop); Ymin(loop+1) = Y;

    Z1 = (1-X)^2+100*(Y-X^2)^2;
    Z  = 2*Z1;

    while abs(Z1-Z)>eps
       % Z update
       Z=Z1; %(1-X)^2+100*(Y-X^2)^2;
       J=derive(X,Y);
       H=derive_second(X,Y);

       % find a new position
       H(1,1)=H(1,1)+lambda;
       H(2,2)=H(2,2)+lambda;
       Delta=-inv(H)*J';
       X1=X+Delta(1);
       Y1=Y+Delta(2);
       Z1=(1-X1)^2+100*(Y1-X1^2)^2;

       if Z1<Z
           lambda=lambda/10;
       else
           while Z1>Z
               lambda=lambda*10;
               H=(J'*J);
               H(1,1)=H(1,1)+lambda;
               H(2,2)=H(2,2)+lambda;
               Delta=-inv(H)*J';
               X1=X+Delta(1);
               Y1=Y+Delta(2);
               Z1=(1-X1)^2+100*(Y1-X1^2)^2;
           end
       end

       if ~strcmp(auto,'auto')
           fprintf(1, '      val = %f\n', Z);
           figure(1); hold('on');
           contour(x,y,z,t2); axis('square');
           plot(Xmin(1:loop+1),Ymin(1:loop+1),'.r');
           plot(Xmin(1:loop+1),Ymin(1:loop+1),'k');
           hold('off');
           pause;
       end

       loop = loop + 1;
       Xmin(loop+1) = X1; X = X1;
       Ymin(loop+1) = Y1; Y = Y1;
    end

    % Final plot
    hold('on');
    plot(Xmin(1:loop),Ymin(1:loop),'.r');
    plot(Xmin(1:loop),Ymin(1:loop),'k');
    hold('off');

    fprintf(1, '      loop = %d\n', loop);
    fprintf(1, '      [X,Y,Z] = [%f, %f, %f]\n', X1, Y1, Z1);
    pause; close;

function [val]=Rosenbrock(X,Y)
   val=(1.0-X)^2+100.0*(Y-X^2)^2;

function [vect]=derive(X,Y)
   pas=sqrt(2^-52);
   if abs(X)<pas
       h=pas;
   else
       h=pas*abs(X);
   end

   X1=X+h; X0=X-h; hx=X1-X0;
   ZX1=Rosenbrock(X1,Y);
   ZX0=Rosenbrock(X0,Y);
   DZX=(ZX1-ZX0)/hx;

   if abs(Y)<pas
       h=pas;
   else
       h=pas*abs(Y);
   end

   Y1=Y+h; Y0=Y-h; hy=Y1-Y0;
   ZY1=Rosenbrock(X,Y1);
   ZY0=Rosenbrock(X,Y0);
   DZY=(ZY1-ZY0)/hy;
   vect=[DZX,DZY];

function [mat]=derive_second(X,Y)
   pas=sqrt(2^-52);
   if abs(X)<pas
       h=pas;
   else
       h=pas*abs(X);
   end

   X1=X+h; X0=X-h; hx=X1-X0;
   vect1=derive(X1,Y);
   vect0=derive(X0,Y);
   vectX=(vect1-vect0)/hx;

   if abs(Y)<pas
       h=pas;
   else
       h=pas*abs(Y);
   end

   Y1=Y+h; Y0=Y-h; hy=Y1-Y0;
   vect1=derive(X,Y1);
   vect0=derive(X,Y0);
   vectY=(vect1-vect0)/hy;
   mat=[vectX;vectY];