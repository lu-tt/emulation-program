function [x,dx] = rk4_1(h,t,x,u) 

   
     k1=Model1(t,x,u);
     dx=k1;
  
     xx=x+k1.*(h/2);  
   
     k2=Model1(t,xx,u);
      
     xx=x+k2.*(h/2);  
   
     k3=Model1(t,xx,u);
      
     xx=x+k3.*h;  
   
     k4=Model1(t,xx,u);
     
     x=x+h/6.*(k1+2.*k2+2.*k3+k4);
end