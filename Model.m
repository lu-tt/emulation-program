function dx=Model(t,x,u)

      dx=zeros(2,1);
      dx(1) = x(2);
      dx(2)=-(1+cos(x(2)))*cos(x(1))+(2+t^2+atan(0.5*x(1)))*u;
