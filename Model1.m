function dx=Model1(t,x,u)

      dx=zeros(2,1);
      dx(1) = x(2);
      dx(2) = -(1+cos(x(1)))*sin(x(2))+(1+0.1*sin(x(1)))*u;
