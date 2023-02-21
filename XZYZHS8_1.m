function xt=XZYZHS8_1(b,p,q,t)
global d; 



d=0.5;



if t>=0&&t<=d
    xt=b*t^2+p*t+q;
else
    xt=0;
end
end

    