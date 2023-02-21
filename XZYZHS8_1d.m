function xt=XZYZHS8_1d(b,p,q,t)
    global d; 
    d=0.5;

    if t>=0&&t<=d
        xt=2*b*t+p;
    else
        xt=0;
    end
end
