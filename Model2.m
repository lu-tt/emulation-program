function dx=Model2(t,x,u)

      dx=zeros(2,1);
      dx(1) = x(2);
      dx(2)=0.2*x(2)/(1+x(2)^2)+sin(t)+(1.3+0.1*sin(x(1))+0.05*cos(x(2)))*u;
