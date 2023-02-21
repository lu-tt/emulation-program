function dx=Model3(t,x,u)

      dx=zeros(2,1);
      dx(1) = x(2);
      dx(2)=3*tanh(x(1))+0.1*sin(x(2))^2+(2+0.1*cos(x(2))^2)*u;
