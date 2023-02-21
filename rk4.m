function [x,dx] = rk4(h,t,x,u) 

     k1=Model(t,x,u);
     dx=k1;
  
     xx=x+k1.*(h/2);  
  
     k2=Model(t,xx,u);
      
     xx=x+k2.*(h/2);  
  
     k3=Model(t,xx,u);
      
     xx=x+k3.*h;  
   
     k4=Model(t,xx,u);
     
     x=x+h/6.*(k1+2.*k2+2.*k3+k4);
end