clc;
clear all;
close all;
global d; 
d=0.5;%the length of Specified interval

A=1;
F=1;

kmax=30;%number of iterations
t0=0;tf=2;%running time interval
h=0.0001;
dt=h;
np=(tf-t0)/h+1;
ts1=0.01/h;
global C
C=8;
B=5;

%communication parameters of multi-agent system
a12=0.9;%relationship between agent 1 and other agents
a13=0.6;
s1=1;

a21=0.9;%relationship between agent 2 and other agents
a23=0.5;

a31=0.6;%relationship between agent 3 and other agents
a32=0.5;
a34=0.4;

a43=0.4;%relationship between agent 4 and other agents
s4=0.8;

M=[s1+a12+a13 -a12 -a13 0;-a21 a21+a23 -a23 0;-a31 -a32 a31+a32+a34 -a34; 0 0 -a43 s4+a43];%matrix M
Moi=inv(M);%the inverse of matrix M

deta1=zeros(kmax,np);
deta2=zeros(kmax,np);
deta3=zeros(kmax,np);
deta4=zeros(kmax,np);
A1_u=zeros(kmax,np);%control input of agent 1
A1_xk1=zeros(kmax,np);%actual trajectory of agent 1
A1_xk2=zeros(kmax,np);%actual velocity of agent 1
A1_ek=zeros(kmax,np);%error between actual trajectory of agent 1 and the desired trajectory
A1_dek=zeros(kmax,np);%derivative of error between actual trajectory of agent 1 and the desired trajectory
A1_ddek=zeros(kmax,np);%second derivative of error between actual trajectory of agent 1 and the desired trajectory
A1_2_ek=zeros(kmax,np);%relative error of actual trajectory between agent 1 and agent 2
A1_2_dek=zeros(kmax,np);%derivative of relative error of actual trajectory between agent 1 and agent 2
A1_2_ddek=zeros(kmax,np);%second derivative of relative error of actual trajectory between agent 1 and agent 2
A1_3_ek=zeros(kmax,np);%relative error of actual trajectory between agent 1 and agent 3
A1_3_dek=zeros(kmax,np);%derivative of relative error of actual trajectory between agent 1 and agent 3
A1_3_ddek=zeros(kmax,np);%second derivative of relative error of actual trajectory between agent 1 and agent 3

A2_u=zeros(kmax,np);%the following variables and their meanings are similar to those above
A2_xk1=zeros(kmax,np);
A2_xk2=zeros(kmax,np);
A2_ek=zeros(kmax,np);
A2_dek=zeros(kmax,np);
A2_ddek=zeros(kmax,np);
A2_1_ek=zeros(kmax,np);
A2_1_dek=zeros(kmax,np);
A2_1_ddek=zeros(kmax,np);
A2_3_ek=zeros(kmax,np);
A2_3_dek=zeros(kmax,np);
A2_3_ddek=zeros(kmax,np);

A3_u=zeros(kmax,np);
A3_xk1=zeros(kmax,np);
A3_xk2=zeros(kmax,np);
A3_ek=zeros(kmax,np);
A3_dek=zeros(kmax,np);
A3_ddek=zeros(kmax,np);
A3_1_ek=zeros(kmax,np);
A3_1_dek=zeros(kmax,np);
A3_1_ddek=zeros(kmax,np);
A3_2_ek=zeros(kmax,np);
A3_2_dek=zeros(kmax,np);
A3_2_ddek=zeros(kmax,np);
A3_4_ek=zeros(kmax,np);
A3_4_dek=zeros(kmax,np);
A3_4_ddek=zeros(kmax,np);

A4_u=zeros(kmax,np);
A4_xk1=zeros(kmax,np);
A4_xk2=zeros(kmax,np);
A4_ek=zeros(kmax,np);
A4_dek=zeros(kmax,np);
A4_ddek=zeros(kmax,np);
A4_3_ek=zeros(kmax,np);
A4_3_dek=zeros(kmax,np);
A4_3_ddek=zeros(kmax,np);

A1_xk=zeros(2,np);%It is mainly used to record the actual track information of agent 1
A1_dxk=zeros(2,np);%It mainly records the actual velocity of Agent 1. The followings are similar.
A2_xk=zeros(2,np);
A2_dxk=zeros(2,np);
A3_xk=zeros(2,np);
A3_dxk=zeros(2,np);
A4_xk=zeros(2,np);
A4_dxk=zeros(2,np);
kesi_1_ek=zeros(kmax,np);%consensus error related to agent 1
kesi_2_ek=zeros(kmax,np);
kesi_3_ek=zeros(kmax,np);
kesi_4_ek=zeros(kmax,np);
kesi_1_dek=zeros(kmax,np);%derivative of consensus error related to agent 1
kesi_2_dek=zeros(kmax,np);
kesi_3_dek=zeros(kmax,np);
kesi_4_dek=zeros(kmax,np);
p1=zeros(kmax,np);%learning parameter  in control law
p2=zeros(kmax,np);
p3=zeros(kmax,np);
v1=zeros(kmax,np);%variables in control law
v2=zeros(kmax,np);
v3=zeros(kmax,np);
b1=zeros(kmax,4);% the parameter in the rectify function
b2=zeros(kmax,4);
b3=zeros(kmax,4);
r0=10;%the ILC gain
r1=10;
r2=10;
B_deta=zeros(1,4);
B_t=zeros(1,np);

time=zeros(1,np);
xd=zeros(1,np);
dxd=zeros(1,np);
ddxd=zeros(1,np);
xz1=zeros(kmax,np);    
dxz1=zeros(kmax,np);
xz2=zeros(kmax,np);    
dxz2=zeros(kmax,np);
xz3=zeros(kmax,np);    
dxz3=zeros(kmax,np);
xz4=zeros(kmax,np);    
dxz4=zeros(kmax,np);
for t=1:1:np
    time(t)=t0+(t-1)*h;
    t1=time(t);
    xd(t)=A*cos(pi*F*t1);%the desired trajectory
    dxd(t)=-A*pi*F*sin(pi*F*t1);%derivative of the desired trajectory
    ddxd(t)=-A*(pi*F)^2*cos(pi*F*t1);%second derivative of the desired trajectory
end

for k=1:kmax
    %define the initial state of each iteration
    A1_xk1(1,1)=1.5*rand; 
    A1_xk1(k+1,1)=1.5*rand;   
    A1_xk2(1,1)=-1*rand;
    A1_xk2(k+1,1)=-1*rand;
    
    A2_xk1(1,1)=0.8*rand;
    A2_xk1(k+1,1)=0.8*rand;
    A2_xk2(1,1)=-3*rand;
    A2_xk2(k+1,1)=-3*rand;
    
    A3_xk1(1,1)=-2*rand;
    A3_xk1(k+1,1)=-2*rand;
    A3_xk2(1,1)=2*rand;
    A3_xk2(k+1,1)=2*rand;
    
    A4_xk1(1,1)=-0.5*rand;
    A4_xk1(k+1,1)=-0.5*rand;
    A4_xk2(1,1)=5*rand;
    A4_xk2(k+1,1)=5*rand;
    
    %errors of agent 1 at time 0  
    A1_ek(k,1)=A1_xk1(k,1)-xd(1);
    A1_2_ek(k,1)=A1_xk1(k,1)-A2_xk1(k,1);
    A1_3_ek(k,1)=A1_xk1(k,1)-A3_xk1(k,1);
    A1_dek(k,1)=A1_xk2(k,1)-dxd(1);
    A1_2_dek(k,1)=A1_xk2(k,1)-A2_xk2(k,1);    
    A1_3_dek(k,1)=A1_xk2(k,1)-A3_xk2(k,1);
    
     %errors of agent 2 at time 0   
    A2_ek(k,1)=A2_xk1(k,1)-xd(1);
    A2_1_ek(k,1)=A2_xk1(k,1)-A1_xk1(k,1);
    A2_3_ek(k,1)=A2_xk1(k,1)-A3_xk1(k,1);
    A2_dek(k,1)=A2_xk2(k,1)-dxd(1);
    A2_1_dek(k,1)=A2_xk2(k,1)-A1_xk2(k,1);
    A2_3_dek(k,1)=A2_xk2(k,1)-A3_xk2(k,1);
    
     %errors of agent 3 at time 0  
    A3_ek(k,1)=A3_xk1(k,1)-xd(1);
    A3_1_ek(k,1)=A3_xk1(k,1)-A1_xk1(k,1);
    A3_2_ek(k,1)=A3_xk1(k,1)-A2_xk1(k,1);
    A3_4_ek(k,1)=A3_xk1(k,1)-A4_xk1(k,1);
    A3_dek(k,1)=A3_xk2(k,1)-dxd(1);
    A3_1_dek(k,1)=A3_xk2(k,1)-A1_xk2(k,1);
    A3_2_dek(k,1)=A3_xk2(k,1)-A2_xk2(k,1);
    A3_4_dek(k,1)=A3_xk2(k,1)-A4_xk2(k,1);
    
     %errors of agent 4 at time 0  
    A4_ek(k,1)=A4_xk1(k,1)-xd(1);
    A4_3_ek(k,1)=A4_xk1(k,1)-A3_xk1(k,1);
    A4_dek(k,1)=A4_xk2(k,1)-dxd(1);
    A4_3_dek(k,1)=A4_xk2(k,1)-A3_xk2(k,1);
    
    
    %derivative of errors related to agent 1 at time 0
    A1_ddek(k,1)=(-(1+cos(A1_xk2(k,1)))*cos(A1_xk1(k,1))+(2+time(1)^2+atan(0.5*A1_xk1(k,1)))*A1_u(k,1))-ddxd(1);
    A1_2_ddek(k,1)=(-(1+cos(A1_xk2(k,1)))*cos(A1_xk1(k,1))+(2+time(1)^2+atan(0.5*A1_xk1(k,1)))*A1_u(k,1))-(-(1+cos(A2_xk1(k,1)))*sin(A2_xk2(k,1))+(1+0.1*sin(A2_xk1(k,1)))*A2_u(k,1));
    A1_3_ddek(k,1)=(-(1+cos(A1_xk2(k,1)))*cos(A1_xk1(k,1))+(2+time(1)^2+atan(0.5*A1_xk1(k,1)))*A1_u(k,1))-(0.2*A3_xk2(k,1)/(1+(A3_xk2(k,1))^2)+sin(time(1))+(1.3+0.1*sin(A3_xk1(k,1))+0.05*cos(A3_xk2(k,1)))*A3_u(k,1));
    
    %derivative of errors related to agent 2 at time 0
    A2_ddek(k,1)=(-(1+cos(A2_xk1(k,1)))*sin(A2_xk2(k,1))+(1+0.1*sin(A2_xk1(k,1)))*A2_u(k,1))-ddxd(1);
    A2_1_ddek(k,1)=(-(1+cos(A2_xk1(k,1)))*sin(A2_xk2(k,1))+(1+0.1*sin(A2_xk1(k,1)))*A2_u(k,1))-(-(1+cos(A1_xk2(k,1)))*cos(A1_xk1(k,1))+(2+time(1)^2+atan(0.5*A1_xk1(k,1)))*A1_u(k,1));
    A2_3_ddek(k,1)=(-(1+cos(A2_xk1(k,1)))*sin(A2_xk2(k,1))+(1+0.1*sin(A2_xk1(k,1)))*A2_u(k,1))-(0.2*A3_xk2(k,1)/(1+(A3_xk2(k,1))^2)+sin(time(1))+(1.3+0.1*sin(A3_xk1(k,1))+0.05*cos(A3_xk2(k,1)))*A3_u(k,1));
    
    %derivative of errors related to agent 3 at time 0
    A3_ddek(k,1)=(0.2*A3_xk2(k,1)/(1+(A3_xk2(k,1))^2)+sin(time(1))+(1.3+0.1*sin(A3_xk1(k,1))+0.05*cos(A3_xk2(k,1)))*A3_u(k,1))-ddxd(1);
    A3_1_ddek(k,1)=(0.2*A3_xk2(k,1)/(1+(A3_xk2(k,1))^2)+sin(time(1))+(1.3+0.1*sin(A3_xk1(k,1))+0.05*cos(A3_xk2(k,1)))*A3_u(k,1))-(-(1+cos(A1_xk2(k,1)))*cos(A1_xk1(k,1))+(2+time(1)^2+atan(0.5*A1_xk1(k,1)))*A1_u(k,1));
    A3_2_ddek(k,1)=(0.2*A3_xk2(k,1)/(1+(A3_xk2(k,1))^2)+sin(time(1))+(1.3+0.1*sin(A3_xk1(k,1))+0.05*cos(A3_xk2(k,1)))*A3_u(k,1))-(-(1+cos(A2_xk1(k,1)))*sin(A2_xk2(k,1))+(1+0.1*sin(A2_xk1(k,1)))*A2_u(k,1));
    A3_4_ddek(k,1)=(0.2*A3_xk2(k,1)/(1+(A3_xk2(k,1))^2)+sin(time(1))+(1.3+0.1*sin(A3_xk1(k,1))+0.05*cos(A3_xk2(k,1)))*A3_u(k,1))-(3*tanh(A4_xk1(k,1))+0.1*(sin(A4_xk2(k,1)))^2+(2+0.1*(cos(A4_xk2(k,1)))^2)*A4_u(k,1));
    
    %derivative of errors related to agent 4 at time 0
    A4_ddek(k,1)=(3*tanh(A4_xk1(k,1))+0.1*(sin(A4_xk2(k,1)))^2+(2+0.1*(cos(A4_xk2(k,1)))^2)*A4_u(k,1))-ddxd(1);
    A4_3_ddek(k,1)=(3*tanh(A4_xk1(k,1))+0.1*(sin(A4_xk2(k,1)))^2+(2+0.1*(cos(A4_xk2(k,1)))^2)*A4_u(k,1))-(0.2*A3_xk2(k,1)/(1+(A3_xk2(k,1))^2)+sin(time(1))+(1.3+0.1*sin(A3_xk1(k,1))+0.05*cos(A3_xk2(k,1)))*A3_u(k,1));
    
end


for k=2:kmax   %k represents iteration axis
    
    A1_xk(1,1)=A1_xk1(k,1);
    A1_xk(2,1)=A1_xk2(k,1);
    
    A2_xk(1,1)=A2_xk1(k,1);
    A2_xk(2,1)=A2_xk2(k,1);
    
    A3_xk(1,1)=A3_xk1(k,1);
    A3_xk(2,1)=A3_xk2(k,1);
    
    A4_xk(1,1)=A4_xk1(k,1);
    A4_xk(2,1)=A4_xk2(k,1);
    
    
    for t=1:1:np-1 %t represents the time axis, this section gives the controller u
    A1_xk(1,t)=A1_xk1(k,t);
    A1_xk(2,t)=A1_xk2(k,t);
    
    A2_xk(1,t)=A2_xk1(k,t);
    A2_xk(2,t)=A2_xk2(k,t);
    
    A3_xk(1,t)=A3_xk1(k,t);
    A3_xk(2,t)=A3_xk2(k,t);
    
    A4_xk(1,t)=A4_xk1(k,t);
    A4_xk(2,t)=A4_xk2(k,t);
        time(t)=t0+(t-1)*h;   %Convert discrete times to actual time, and continuous time cannot be achieved during simulation.
        t1=time(t);
        me=M*[A1_ek(k,t);A2_ek(k,t);A3_ek(k,t);A4_ek(k,t)];
        kesi_1_ek(k,t)=me(1,1);
        kesi_2_ek(k,t)=me(2,1);
        kesi_3_ek(k,t)=me(3,1);
        kesi_4_ek(k,t)=me(4,1);
        me=M*[A1_dek(k,t);A2_dek(k,t);A3_dek(k,t);A4_dek(k,t)];
        kesi_1_dek(k,t)=me(1,1);
        kesi_2_dek(k,t)=me(2,1);
        kesi_3_dek(k,t)=me(3,1);
        kesi_4_dek(k,t)=me(4,1);
       
        A=inv([1/3*d^3 1/2*d^2 d;d^2  d 1;0 0 1])*[kesi_1_ek(k,1)/exp(kesi_1_ek(k,1));0;-(kesi_1_dek(k,1)+C*kesi_1_ek(k,1))/exp(kesi_1_ek(k,1))];%find the coefficient of the polynomial that rectifys the state shifts of agent 1
        b1(k,1)=A(1,1);
        b2(k,1)=A(2,1);
        b3(k,1)=A(3,1);
        A=inv([1/3*d^3 1/2*d^2 d;d^2  d 1;0 0 1])*[kesi_2_ek(k,1)/exp(kesi_2_ek(k,1));0;-(kesi_2_dek(k,1)+C*kesi_2_ek(k,1))/exp(kesi_2_ek(k,1))];
        b1(k,2)=A(1,1);
        b2(k,2)=A(2,1);
        b3(k,2)=A(3,1);
        A=inv([1/3*d^3 1/2*d^2 d;d^2  d 1;0 0 1])*[kesi_3_ek(k,1)/exp(kesi_3_ek(k,1));0;-(kesi_3_dek(k,1)+C*kesi_3_ek(k,1))/exp(kesi_3_ek(k,1))];
        b1(k,3)=A(1,1);
        b2(k,3)=A(2,1);
        b3(k,3)=A(3,1);
        A=inv([1/3*d^3 1/2*d^2 d;d^2  d 1;0 0 1])*[kesi_4_ek(k,1)/exp(kesi_4_ek(k,1));0;-(kesi_4_dek(k,1)+C*kesi_4_ek(k,1))/exp(kesi_4_ek(k,1))];
        b1(k,4)=A(1,1);
        b2(k,4)=A(2,1);
        b3(k,4)=A(3,1);
        if t<d/h  %[0, d/h] is the rectifying interval
            xz1(k,t)=XZYZHS8_1(b1(k,1),b2(k,1),b3(k,1),(t-1)*dt)*exp(-C*(t-1)*dt+kesi_1_ek(k,1));  % rectification amount of agent 1      
            dxz1(k,t)=XZYZHS8_1d(b1(k,1),b2(k,1),b3(k,1),(t-1)*dt)*exp(-C*(t-1)*dt+kesi_1_ek(k,1))-C*XZYZHS8_1(b1(k,1),b2(k,1),b3(k,1),(t-1)*dt)*exp(-C*(t-1)*dt+kesi_1_ek(k,1)) ;%derivative of rectification amount of agent 1  
        	deta1(k,t)=kesi_1_dek(k,t)+C*kesi_1_ek(k,t)+xz1(k,t); %the sliding mode error function
            v1(k,t)=(2+0.5)*abs(A1_ek(k,t))+(1+abs(C))*abs(A1_dek(k,t));
            p1(k,t)=SAT1_1(p1(k-1,t))-r0*deta1(k,t); %learning update law of the first parameter
            p1(k,t)=SAT1_1(p1(k,t));
            p2(k,t)=SAT1_2(p2(k-1,t))+r1*abs(deta1(k,t));%learning update law of the second parameter
            p2(k,t)=SAT1_2(p2(k,t));
            p3(k,t)=SAT1_3(p3(k-1,t))+r2*deta1(k,t);%learning update law of the third parameter
            p3(k,t)=SAT1_3(p3(k,t));            
            A1_u(k,t)=p1(k,t)-0.4*v1(k,t)*atan(k*k*deta1(k,t)*v1(k,t))-0.4*p2(k,t)*atan(k*k*deta1(k,t)*p2(k,t))-0.4*p3(k,t)*atan(k*k*deta1(k,t)*p3(k,t))-B*deta1(k,t);%control law
        else
        	deta1(k,t)=kesi_1_dek(k,t)+C*kesi_1_ek(k,t); 
            v1(k,t)=(2+0.5)*abs(A1_ek(k,t))+(1+abs(C))*abs(A1_dek(k,t));
            p1(k,t)=SAT1_1(p1(k-1,t))-r0*deta1(k,t); 
            p1(k,t)=SAT1_1(p1(k,t));
            p2(k,t)=SAT1_2(p2(k-1,t))+r1*deta1(k,t);
            p2(k,t)=SAT1_2(p2(k,t));
            A1_u(k,t)=p1(k,t)-0.4*v1(k,t)*atan(k*k*deta1(k,t)*v1(k,t))-0.4*p2(k,t)*atan(k*k*deta1(k,t)*p2(k,t))-0.4*p3(k,t)*atan(k*k*deta1(k,t)*p3(k,t))-B*deta1(k,t);
        end
        [A1_xk(:,t+1),A1_dxk(:,t)]=rk4(h,time(t),A1_xk(:,t),A1_u(k,t));%The function rk4 is used to solve the differential equation by the fourth order Runge Kutta method. There are four input parameters and two output parameters.
        A1_xk1(k,t+1)=A1_xk(1,t+1);%The two output parameters return the actual track and its derivative. If the mathematical model of the agent changes, you only need to change the information in the file Model. m.
        A1_xk2(k,t+1)=A1_xk(2,t+1);
        
        if t<d/h  %The following is the case of  agent 2.
             xz2(k,t)=XZYZHS8_1(b1(k,2),b2(k,2),b3(k,2),(t-1)*dt)*exp(-C*(t-1)*dt+kesi_2_ek(k,1));        
             dxz2(k,t)=XZYZHS8_1d(b1(k,2),b2(k,2),b3(k,2),(t-1)*dt)*exp(-C*(t-1)*dt+kesi_2_ek(k,1))-C*XZYZHS8_1(b1(k,2),b2(k,2),b3(k,2),(t-1)*dt)*exp(-C*(t-1)*dt+kesi_2_ek(k,1));
        	deta2(k,t)=kesi_2_dek(k,t)+C*kesi_2_ek(k,t)+xz2(k,t); 
            v1(k,t)=(1+0.1)*abs(A2_ek(k,t))+(2+abs(C))*abs(A2_dek(k,t));
            p1(k,t)=SAT1_1(p1(k-1,t))-r0*deta2(k,t); 
            p1(k,t)=SAT1_1(p1(k,t));
            p2(k,t)=SAT1_2(p2(k-1,t))+r1*abs(deta2(k,t));
            p2(k,t)=SAT1_2(p2(k,t));
            p3(k,t)=SAT1_3(p3(k-1,t))+r2*deta2(k,t);
            p3(k,t)=SAT1_3(p3(k,t));
            A2_u(k,t)=p1(k,t)-0.9*v1(k,t)*atan(k*k*deta2(k,t)*v1(k,t))-0.9*p2(k,t)*atan(k*k*deta2(k,t)*p2(k,t))-0.9*p3(k,t)*atan(k*k*deta2(k,t)*p3(k,t))-B*deta2(k,t);%
        else
        	deta2(k,t)=kesi_2_dek(k,t)+C*kesi_2_ek(k,t); 
            v1(k,t)=(1+0.1)*abs(A2_ek(k,t))+(2+abs(C))*abs(A2_dek(k,t));
            p1(k,t)=SAT1_1(p1(k-1,t))-r0*deta2(k,t);  
            p1(k,t)=SAT1_1(p1(k,t));
            p2(k,t)=SAT1_2(p2(k-1,t))+r1*abs(deta2(k,t));
            p2(k,t)=SAT1_2(p2(k,t));
            p3(k,t)=SAT1_3(p3(k-1,t))+r2*deta2(k,t);
            p3(k,t)=SAT1_3(p3(k,t));
            A2_u(k,t)=p1(k,t)-0.9*v1(k,t)*atan(k*k*deta2(k,t)*v1(k,t))-0.9*p2(k,t)*atan(k*k*deta2(k,t)*p2(k,t))-0.9*p3(k,t)*atan(k*k*deta2(k,t)*p3(k,t))-B*deta2(k,t);
        end
        [A2_xk(:,t+1),A2_dxk(:,t)]=rk4_1(h,time(t),A2_xk(:,t),A2_u(k,t));
        A2_xk1(k,t+1)=A2_xk(1,t+1);
        A2_xk2(k,t+1)=A2_xk(2,t+1);
        
        if t<d/h  %The following is the case of  agent 3.
             xz3(k,t)=XZYZHS8_1(b1(k,3),b2(k,3),b3(k,3),(t-1)*dt)*exp(-C*(t-1)*dt+kesi_3_ek(k,1));        
             dxz3(k,t)=XZYZHS8_1d(b1(k,3),b2(k,3),b3(k,3),(t-1)*dt)*exp(-C*(t-1)*dt+kesi_3_ek(k,1))-C*XZYZHS8_1(b1(k,3),b2(k,3),b3(k,3),(t-1)*dt)*exp(-C*(t-1)*dt+kesi_3_ek(k,1));
        	deta3(k,t)=kesi_3_dek(k,t)+C*kesi_3_ek(k,t)+xz3(k,t); 
            v1(k,t)=0.1*abs(A3_ek(k,t))+(0.2+0.05+abs(C))*abs(A3_dek(k,t));
            p1(k,t)=SAT1_1(p1(k-1,t))-r0*deta3(k,t);  
            p1(k,t)=SAT1_1(p1(k,t));
            p2(k,t)=SAT1_2(p2(k-1,t))+r1*abs(deta3(k,t));
            p2(k,t)=SAT1_2(p2(k,t));
            p3(k,t)=SAT1_3(p3(k-1,t))+r2*deta3(k,t);
            p3(k,t)=SAT1_3(p3(k,t));
            A3_u(k,t)=p1(k,t)-1.15*v1(k,t)*atan(k*k*deta3(k,t)*v1(k,t))-1.15*p2(k,t)*atan(k*k*deta3(k,t)*p2(k,t))-1.15*p3(k,t)*atan(k*k*deta3(k,t)*p3(k,t))-B*deta3(k,t);
        else
        	deta3(k,t)=kesi_3_dek(k,t)+C*kesi_3_ek(k,t); 
            v1(k,t)=0.1*abs(A3_ek(k,t))+(0.2+0.05+abs(C))*abs(A3_dek(k,t));
            p1(k,t)=SAT1_1(p1(k-1,t))-r0*deta3(k,t); 
            p1(k,t)=SAT1_1(p1(k,t));
            p2(k,t)=SAT1_2(p2(k-1,t))+r1*abs(deta3(k,t));
            p2(k,t)=SAT1_2(p2(k,t));
            p3(k,t)=SAT1_3(p3(k-1,t))+r2*deta3(k,t);
            p3(k,t)=SAT1_3(p3(k,t));
            A3_u(k,t)=p1(k,t)-1.15*v1(k,t)*atan(k*k*deta3(k,t)*v1(k,t))-1.15*p2(k,t)*atan(k*k*deta3(k,t)*p2(k,t))-1.15*p3(k,t)*atan(k*k*deta3(k,t)*p3(k,t))-B*deta3(k,t);
        end
        [A3_xk(:,t+1),A3_dxk(:,t)]=rk4_2(h,time(t),A3_xk(:,t),A3_u(k,t));        
        A3_xk1(k,t+1)=A3_xk(1,t+1);
        A3_xk2(k,t+1)=A3_xk(2,t+1);
        
        if t<d/h  %The following is the case of  agent 4.
             xz4(k,t)=XZYZHS8_1(b1(k,4),b2(k,4),b3(k,4),(t-1)*dt)*exp(-C*(t-1)*dt+kesi_4_ek(k,1));        
             dxz4(k,t)=XZYZHS8_1d(b1(k,4),b2(k,4),b3(k,4),(t-1)*dt)*exp(-C*(t-1)*dt+kesi_4_ek(k,1))-C*XZYZHS8_1(b1(k,4),b2(k,4),b3(k,4),(t-1)*dt)*exp(-C*(t-1)*dt+kesi_4_ek(k,1));
        	deta4(k,t)=kesi_4_dek(k,t)+C*kesi_4_ek(k,t)+xz4(k,t); 
            v1(k,t)=3*abs(A4_ek(k,t))+(0.2+0.2+abs(C))*abs(A4_dek(k,t));
            p1(k,t)=SAT1_1(p1(k-1,t))-r0*deta4(k,t); 
            p1(k,t)=SAT1_1(p1(k,t));
            p2(k,t)=SAT1_2(p2(k-1,t))+r1*abs(deta4(k,t));
            p2(k,t)=SAT1_2(p2(k,t));
            p3(k,t)=SAT1_3(p3(k-1,t))+r2*deta4(k,t);
            p3(k,t)=SAT1_3(p3(k,t));
            A4_u(k,t)=p1(k,t)-2*v1(k,t)*atan(k*k*deta4(k,t)*v1(k,t))-2*p2(k,t)*atan(k*k*deta4(k,t)*p2(k,t))-2*p3(k,t)*atan(k*k*deta4(k,t)*p3(k,t))-B*deta4(k,t);
        else
        	deta4(k,t)=kesi_4_dek(k,t)+C*kesi_4_ek(k,t); 
            v1(k,t)=3*abs(A4_ek(k,t))+(0.2+0.2+abs(C))*abs(A4_dek(k,t));
            p1(k,t)=SAT1_1(p1(k-1,t))-r0*deta4(k,t); 
            p1(k,t)=SAT1_1(p1(k,t));
            p2(k,t)=SAT1_2(p2(k-1,t))+r1*abs(deta4(k,t));
            p2(k,t)=SAT1_2(p2(k,t));
            p3(k,t)=SAT1_3(p3(k-1,t))+r2*deta4(k,t);
            p3(k,t)=SAT1_3(p3(k,t));
            A4_u(k,t)=p1(k,t)-2*v1(k,t)*atan(k*k*deta4(k,t)*v1(k,t))-2*p2(k,t)*atan(k*k*deta4(k,t)*p2(k,t))-2*p3(k,t)*atan(k*k*deta4(k,t)*p3(k,t))-B*deta4(k,t);
        end        
        [A4_xk(:,t+1),A4_dxk(:,t)]=rk4_3(h,time(t),A4_xk(:,t),A4_u(k,t));
        A4_xk1(k,t+1)=A4_xk(1,t+1);
        A4_xk2(k,t+1)=A4_xk(2,t+1);
        
        
    %errors of agent 1 at time t    
    A1_ek(k,t+1)=A1_xk1(k,t+1)-xd(t+1);
    A1_dek(k,t+1)=A1_xk2(k,t+1)-dxd(t+1);
    A1_2_ek(k,t+1)=A1_xk1(k,t+1)-A2_xk1(k,t+1);
    A1_2_dek(k,t+1)=A1_xk2(k,t+1)-A2_xk2(k,t+1);
    A1_3_ek(k,t+1)=A1_xk1(k,t+1)-A3_xk1(k,t+1);
    A1_3_dek(k,t+1)=A1_xk2(k,t+1)-A3_xk2(k,t+1);
     %errors of agent 2 at time t 
    A2_ek(k,t+1)=A2_xk1(k,t+1)-xd(t+1);
    A2_1_ek(k,t+1)=A2_xk1(k,t+1)-A1_xk1(k,t+1);
    A2_3_ek(k,t+1)=A2_xk1(k,t+1)-A3_xk1(k,t+1);
    A2_dek(k,t+1)=A2_xk2(k,t+1)-dxd(t+1);
    A2_1_dek(k,t+1)=A2_xk2(k,t+1)-A1_xk2(k,t+1);
    A2_3_dek(k,t+1)=A2_xk2(k,t+1)-A3_xk2(k,t+1);
    
     %errors of agent 3 at time t  
    A3_ek(k,t+1)=A3_xk1(k,t+1)-xd(t+1);
    A3_1_ek(k,t+1)=A3_xk1(k,t+1)-A1_xk1(k,t+1);
    A3_2_ek(k,t+1)=A3_xk1(k,t+1)-A2_xk1(k,t+1);
    A3_4_ek(k,t+1)=A3_xk1(k,t+1)-A4_xk1(k,t+1);
    A3_dek(k,t+1)=A3_xk2(k,t+1)-dxd(t+1);
    A3_1_dek(k,t+1)=A3_xk2(k,t+1)-A1_xk2(k,t+1);
    A3_2_dek(k,t+1)=A3_xk2(k,t+1)-A2_xk2(k,t+1);
    A3_4_dek(k,t+1)=A3_xk2(k,t+1)-A4_xk2(k,t+1);
    
     %errors of agent 4 at time t   
    A4_ek(k,t+1)=A4_xk1(k,t+1)-xd(t+1);
    A4_3_ek(k,t+1)=A4_xk1(k,t+1)-A3_xk1(k,t+1);
    A4_dek(k,t+1)=A4_xk2(k,t+1)-dxd(t+1);
    A4_3_dek(k,t+1)=A4_xk2(k,t+1)-A3_xk2(k,t+1);
       
            
    end
    for t=1:1:np-1
     B_deta(1)=deta1(k,t);
     B_deta(2)=deta2(k,t);
     B_deta(3)=deta3(k,t);
     B_deta(4)=deta4(k,t);
     B_t(k,t)=0.5*B_deta*Moi*B_deta';%energy function V
    end      
      
    
       
    k   
    
    
end
figure_FontSize=22;%The following outputs the simulation results.
figure_size=1;
font_size=22;

figure(1);
subplot(211);
hold on;
plot(time(:),xd,'k',time(:),A1_xk1(kmax,:),'g' ,time(:),A2_xk1(kmax,:),'b',time(:),A3_xk1(kmax,:),'r',time(:),A4_xk1(kmax,:),'m');
h1=xlabel('$t(s)$','Interpreter','latex');ylabel('$x_{j,k}, x_{0}$','Interpreter','latex');
set(get(gca,'XLabel'),'FontSize',figure_FontSize);
set(get(gca,'YLabel'),'FontSize',figure_FontSize); 
set(gca,'fontsize',font_size);
set(h1,'Fontsize',figure_FontSize); 
legend({'Leader','Agent 1','Agent 2','Agent 3','Agent 4'},'FontSize',10);

subplot(212);
hold on;
plot(time(:),dxd,'k',time(:),A1_xk2(kmax,:),'g' ,time(:),A2_xk2(kmax,:),'b',time(:),A3_xk2(kmax,:),'r',time(:),A4_xk2(kmax,:),'m');
xlabel('$t(s)$','Interpreter','latex'),ylabel('$\dot{x}_{j,k}, \dot{x}_{0}$','Interpreter','latex');
set(get(gca,'XLabel'),'FontSize',figure_FontSize);
set(get(gca,'YLabel'),'FontSize',figure_FontSize); 
set(gca,'fontsize',font_size);
legend({'Leader','Agent 1','Agent 2','Agent 3','Agent 4'},'FontSize',10);

figure(2);
subplot(211);
hold on;
plot(time(:),A1_ek(kmax,:),'g',time(:),A2_ek(kmax,:),'b',time(:),A3_ek(kmax,:),'r',time(:),A4_ek(kmax,:),'m');
xlabel('$t(s)$','Interpreter','latex'),ylabel('$e_{j,k}$','Interpreter','latex');
set(get(gca,'XLabel'),'FontSize',figure_FontSize);
set(get(gca,'YLabel'),'FontSize',figure_FontSize); 
set(gca,'fontsize',font_size);
legend({'$e_{1,k}$','$e_{2,k}$','$e_{3,k}$','$e_{4,k}$'},'Interpreter','latex','FontSize',15);

subplot(212);
hold on;
plot(time(:),A1_dek(kmax,:),'g',time(:),A2_dek(kmax,:),'b',time(:),A3_dek(kmax,:),'r',time(:),A4_dek(kmax,:),'m');
xlabel('$t(s)$','Interpreter','latex'),ylabel('$\dot{e}_{j,k}$','Interpreter','latex');
set(get(gca,'XLabel'),'FontSize',figure_FontSize);
set(get(gca,'YLabel'),'FontSize',figure_FontSize); 
set(gca,'fontsize',font_size);
legend({'$\dot{e}_{1,k}$','$\dot{e}_{2,k}$','$\dot{e}_{3,k}$','$\dot{e}_{4,k}$'},'Interpreter','latex','FontSize',15);

figure(3);
subplot(221)
plot(time(:),xd,'k',time(:),A1_xk1(kmax-2,:),'g',time(:),A1_xk1(kmax-1,:),'b',time(:),A1_xk1(kmax,:),'r');
xlabel('$t(s)$','Interpreter','latex'),ylabel('$x_{1,k}, x_{0}$','Interpreter','latex');
set(get(gca,'XLabel'),'FontSize',figure_FontSize);
set(get(gca,'YLabel'),'FontSize',figure_FontSize); 
set(gca,'fontsize',font_size);
legend({'$x_0$','$x_{1,28}$','$x_{1,29}$','$x_{1,30}$'},'Interpreter','latex','FontSize',15);

subplot(222)
plot(time(:),xd,'k',time(:),A2_xk1(kmax-2,:),'g',time(:),A2_xk1(kmax-1,:),'b',time(:),A2_xk1(kmax,:),'r');
xlabel('$t(s)$','Interpreter','latex'),ylabel('$x_{2,k}, x_{0}$','Interpreter','latex');
set(get(gca,'XLabel'),'FontSize',figure_FontSize);
set(get(gca,'YLabel'),'FontSize',figure_FontSize); 
set(gca,'fontsize',font_size);
legend({'$x_0$','$x_{2,28}$','$x_{2,29}$','$x_{2,30}$'},'Interpreter','latex','FontSize',15);

subplot(223)
plot(time(:),xd,'k',time(:),A3_xk1(kmax-2,:),'g',time(:),A3_xk1(kmax-1,:),'b',time(:),A3_xk1(kmax,:),'r');
xlabel('$t(s)$','Interpreter','latex'),ylabel('$x_{3,k}, x_{0}$','Interpreter','latex');
set(get(gca,'XLabel'),'FontSize',figure_FontSize);
set(get(gca,'YLabel'),'FontSize',figure_FontSize); 
set(gca,'fontsize',font_size);
legend({'$x_0$','$x_{3,28}$','$x_{3,29}$','$x_{3,30}$'},'Interpreter','latex','FontSize',15);

subplot(224)
plot(time(:),xd,'k',time(:),A4_xk1(kmax-2,:),'g',time(:),A4_xk1(kmax-1,:),'b',time(:),A4_xk1(kmax,:),'r');
xlabel('$t(s)$','Interpreter','latex'),ylabel('$x_{4,k}, x_{0}$','Interpreter','latex');
set(get(gca,'XLabel'),'FontSize',figure_FontSize);
set(get(gca,'YLabel'),'FontSize',figure_FontSize); 
set(gca,'fontsize',font_size);
legend({'$x_0$','$x_{4,28}$','$x_{4,29}$','$x_{4,30}$'},'Interpreter','latex','FontSize',15);

figure(4);
subplot(221)
plot(time(:),dxd,'k',time(:),A1_xk2(kmax-2,:),'g',time(:),A1_xk2(kmax-1,:),'b',time(:),A1_xk2(kmax,:),'r');
xlabel('$t(s)$','Interpreter','latex'),ylabel('$\dot{x}_{1,k}, \dot{x}_{0}$','Interpreter','latex');
set(get(gca,'XLabel'),'FontSize',figure_FontSize);
set(get(gca,'YLabel'),'FontSize',figure_FontSize); 
set(gca,'fontsize',font_size);
legend({'$\dot{x}_{0}$','$\dot{x}_{2,28}$','$\dot{x}_{2,29}$','$\dot{x}_{2,30}$'},'Interpreter','latex','FontSize',15);

subplot(222)
plot(time(:),dxd,'k',time(:),A2_xk2(kmax-2,:),'g',time(:),A2_xk2(kmax-1,:),'b',time(:),A2_xk2(kmax,:),'r');
xlabel('$t(s)$','Interpreter','latex'),ylabel('$\dot{x}_{2,k}, \dot{x}_{0}$','Interpreter','latex');
set(get(gca,'XLabel'),'FontSize',figure_FontSize);
set(get(gca,'YLabel'),'FontSize',figure_FontSize); 
set(gca,'fontsize',font_size);
legend({'$\dot{x}_{0}$','$\dot{x}_{2,28}$','$\dot{x}_{2,29}$','$\dot{x}_{2,30}$'},'Interpreter','latex','FontSize',15);

subplot(223)
plot(time(:),dxd,'k',time(:),A3_xk2(kmax-2,:),'g',time(:),A3_xk2(kmax-1,:),'b',time(:),A3_xk2(kmax,:),'r');
xlabel('$t(s)$','Interpreter','latex'),ylabel('$\dot{x}_{3,k}, \dot{x}_{0}$','Interpreter','latex');
set(get(gca,'XLabel'),'FontSize',figure_FontSize);
set(get(gca,'YLabel'),'FontSize',figure_FontSize); 
set(gca,'fontsize',font_size);
legend({'$\dot{x}_{0}$','$\dot{x}_{3,28}$','$\dot{x}_{3,29}$','$\dot{x}_{3,30}$'},'Interpreter','latex','FontSize',15);

subplot(224)
plot(time(:),dxd,'k',time(:),A4_xk2(kmax-2,:),'g',time(:),A4_xk2(kmax-1,:),'b',time(:),A4_xk2(kmax,:),'r');
xlabel('$t(s)$','Interpreter','latex'),ylabel('$\dot{x}_{4,k}, \dot{x}_{0}$','Interpreter','latex');
set(get(gca,'XLabel'),'FontSize',figure_FontSize);
set(get(gca,'YLabel'),'FontSize',figure_FontSize); 
set(gca,'fontsize',font_size);
legend({'$\dot{x}_{0}$','$\dot{x}_{4,28}$','$\dot{x}_{4,29}$','$\dot{x}_{4,30}$'},'Interpreter','latex','FontSize',15);

figure(5);
subplot(221)
plot(time(:),A1_ek(kmax-2,:),'g',time(:),A1_ek(kmax-1,:),'b',time(:),A1_ek(kmax,:),'r');
xlabel('$t(s)$','Interpreter','latex'),ylabel('$e_{1,k}$','Interpreter','latex');
set(get(gca,'XLabel'),'FontSize',figure_FontSize);
set(get(gca,'YLabel'),'FontSize',figure_FontSize); 
set(gca,'fontsize',font_size);
legend({'$e_{1,28}$','$e_{1,29}$','$e_{1,30}$'},'Interpreter','latex','FontSize',15);

subplot(222)
plot(time(:),A2_ek(kmax-2,:),'g',time(:),A2_ek(kmax-1,:),'b',time(:),A2_ek(kmax,:),'r');
xlabel('$t(s)$','Interpreter','latex'),ylabel('$e_{2,k}$','Interpreter','latex');
set(get(gca,'XLabel'),'FontSize',figure_FontSize);
set(get(gca,'YLabel'),'FontSize',figure_FontSize); 
set(gca,'fontsize',font_size);
legend({'$e_{2,28}$','$e_{2,29}$','$e_{2,30}$'},'Interpreter','latex','FontSize',15);

subplot(223)
plot(time(:),A3_ek(kmax-2,:),'g',time(:),A3_ek(kmax-1,:),'b',time(:),A3_ek(kmax,:),'r');
xlabel('$t(s)$','Interpreter','latex'),ylabel('$e_{3,k}$','Interpreter','latex');
set(get(gca,'XLabel'),'FontSize',figure_FontSize);
set(get(gca,'YLabel'),'FontSize',figure_FontSize); 
set(gca,'fontsize',font_size);
legend({'$e_{3,28}$','$e_{3,29}$','$e_{3,30}$'},'Interpreter','latex','FontSize',15);

subplot(224)
plot(time(:),A4_ek(kmax-2,:),'g',time(:),A4_ek(kmax-1,:),'b',time(:),A4_ek(kmax,:),'r');
xlabel('$t(s)$','Interpreter','latex'),ylabel('$e_{4,k}$','Interpreter','latex');
set(get(gca,'XLabel'),'FontSize',figure_FontSize);
set(get(gca,'YLabel'),'FontSize',figure_FontSize); 
set(gca,'fontsize',font_size);
legend({'$e_{4,28}$','$e_{4,29}$','$e_{4,30}$'},'Interpreter','latex','FontSize',15);

figure(6);
subplot(221)
plot(time(:),A1_dek(kmax-2,:),'g',time(:),A1_dek(kmax-1,:),'b',time(:),A1_dek(kmax,:),'r');
xlabel('$t(s)$','Interpreter','latex'),ylabel('$\dot{e}_{1,k}$','Interpreter','latex');
set(get(gca,'XLabel'),'FontSize',figure_FontSize);
set(get(gca,'YLabel'),'FontSize',figure_FontSize); 
set(gca,'fontsize',font_size);
legend({'$\dot{e}_{1,28}$','$\dot{e}_{1,29}$','$\dot{e}_{1,30}$'},'Interpreter','latex','FontSize',15);

subplot(222)
plot(time(:),A2_dek(kmax-2,:),'g',time(:),A2_dek(kmax-1,:),'b',time(:),A2_dek(kmax,:),'r');
xlabel('$t(s)$','Interpreter','latex'),ylabel('$\dot{e}_{2,k}$','Interpreter','latex');
set(get(gca,'XLabel'),'FontSize',figure_FontSize);
set(get(gca,'YLabel'),'FontSize',figure_FontSize); 
set(gca,'fontsize',font_size);
legend({'$\dot{e}_{2,28}$','$\dot{e}_{2,29}$','$\dot{e}_{2,30}$'},'Interpreter','latex','FontSize',15);

subplot(223)
plot(time(:),A3_dek(kmax-2,:),'g',time(:),A3_dek(kmax-1,:),'b',time(:),A3_dek(kmax,:),'r');
xlabel('$t(s)$','Interpreter','latex'),ylabel('$\dot{e}_{3,k}$','Interpreter','latex');
set(get(gca,'XLabel'),'FontSize',figure_FontSize);
set(get(gca,'YLabel'),'FontSize',figure_FontSize); 
set(gca,'fontsize',font_size);
legend({'$\dot{e}_{3,28}$','$\dot{e}_{3,29}$','$\dot{e}_{3,30}$'},'Interpreter','latex','FontSize',15);

subplot(224)
plot(time(:),A4_dek(kmax-2,:),'g',time(:),A4_dek(kmax-1,:),'b',time(:),A4_dek(kmax,:),'r');
xlabel('$t(s)$','Interpreter','latex'),ylabel('$\dot{e}_{4,k}$','Interpreter','latex');
set(get(gca,'XLabel'),'FontSize',figure_FontSize);
set(get(gca,'YLabel'),'FontSize',figure_FontSize); 
set(gca,'fontsize',font_size);
legend({'$\dot{e}_{4,28}$','$\dot{e}_{4,29}$','$\dot{e}_{4,30}$'},'Interpreter','latex','FontSize',15);


figure(7);
subplot(211);
hold on;
plot(time(:),kesi_1_ek(kmax,:),'g',time(:),kesi_2_ek(kmax,:),'b',time(:),kesi_3_ek(kmax,:),'r',time(:),kesi_4_ek(kmax,:),'m');
xlabel('$t(s)$','Interpreter','latex'),ylabel('$\xi_{j,k}$','Interpreter','latex');
set(get(gca,'XLabel'),'FontSize',figure_FontSize);
set(get(gca,'YLabel'),'FontSize',figure_FontSize); 
set(gca,'fontsize',font_size);
legend({'$\xi_{1,k}$','$\xi_{2,k}$','$\xi_{3,k}$','$\xi_{4,k}$'},'Interpreter','latex','FontSize',15);
box on;

subplot(212);
hold on;
plot(time(:),kesi_1_dek(kmax,:),'g',time(:),kesi_2_dek(kmax,:),'b',time(:),kesi_3_dek(kmax,:),'r',time(:),kesi_4_dek(kmax,:),'m');
xlabel('$t(s)$','Interpreter','latex'),ylabel('$\dot{\xi}_{j,k}$','Interpreter','latex');
set(get(gca,'XLabel'),'FontSize',figure_FontSize);
set(get(gca,'YLabel'),'FontSize',figure_FontSize); 
set(gca,'fontsize',font_size);
box on;
legend({'$\dot{\xi}_{1,k}$','$\dot{\xi}_{2,k}$','$\dot{\xi}_{3,k}$','$\dot{\xi}_{4,k}$'},'Interpreter','latex','FontSize',15);

figure(8)
subplot(211)
k=2:kmax;
plot(k,abs(A1_ek(k,(np-1)*0.25)),'g',k,abs(A2_ek(k,(np-1)*0.25)),'b',k,abs(A3_ek(k,(np-1)*0.25)),'r',k,abs(A4_ek(k,(np-1)*0.25)),'m');
ylabel('$|e_{j,k}(t_p)|$','Interpreter','latex');
xlabel('k');
set(get(gca,'XLabel'),'FontSize',figure_FontSize);
set(get(gca,'YLabel'),'FontSize',figure_FontSize); 
set(gca,'fontsize',font_size);
legend({'$|e_{1,k}(t_p)|$','$|e_{2,k}(t_p)|$','$|e_{3,k}(t_p)|$','$|e_{4,k}(t_p)|$'},'Interpreter','latex','FontSize',15);

subplot(212)
k=2:kmax;
plot(k,abs(A1_ek(k,(np-1)*0.5)),'g',k,abs(A2_ek(k,(np-1)*0.5)),'b',k,abs(A3_ek(k,(np-1)*0.5)),'r',k,abs(A4_ek(k,(np-1)*0.5)),'m');
ylabel('$|e_{j,k}(2t_p)|$','Interpreter','latex');
xlabel('k');
set(get(gca,'XLabel'),'FontSize',figure_FontSize);
set(get(gca,'YLabel'),'FontSize',figure_FontSize); 
set(gca,'fontsize',font_size);
legend({'$|e_{1,k}(2t_p)|$','$|e_{2,k}(2t_p)|$','$|e_{3,k}(2t_p)|$','$|e_{4,k}(2t_p)|$'},'Interpreter','latex','FontSize',15);


figure(9)
subplot(211)
k=2:kmax;
plot(k,abs(A1_dek(k,(np-1)*0.25)),'g',k,abs(A2_dek(k,(np-1)*0.25)),'b',k,abs(A3_dek(k,(np-1)*0.25)),'r',k,abs(A4_dek(k,(np-1)*0.25)),'m');
ylabel('$|\dot{e}_{j,k}(t_p)|$','Interpreter','latex');
xlabel('k');
set(get(gca,'XLabel'),'FontSize',figure_FontSize);
set(get(gca,'YLabel'),'FontSize',figure_FontSize); 
set(gca,'fontsize',font_size);
legend({'$|\dot{e}_{1,k}(t_p)|$','$|\dot{e}_{2,k}(t_p)|$','$|\dot{e}_{3,k}(t_p)|$','$|\dot{e}_{4,k}(t_p)|$'},'Interpreter','latex','FontSize',15);

subplot(212)
k=2:kmax;
plot(k,abs(A1_dek(k,(np-1)*0.5)),'g',k,abs(A2_dek(k,(np-1)*0.5)),'b',k,abs(A3_dek(k,(np-1)*0.5)),'r',k,abs(A4_dek(k,(np-1)*0.5)),'m');
ylabel('$|\dot{e}_{j,k}(2t_p)|$','Interpreter','latex');
xlabel('k');
set(get(gca,'XLabel'),'FontSize',figure_FontSize);
set(get(gca,'YLabel'),'FontSize',figure_FontSize); 
set(gca,'fontsize',font_size);
legend({'$|\dot{e}_{1,k}(2t_p)|$','$|\dot{e}_{2,k}(2t_p)|$','$|\dot{e}_{3,k}(2t_p)|$','$|\dot{e}_{4,k}(2t_p)|$'},'Interpreter','latex','FontSize',15);

figure(10)
subplot(211)
k=2:kmax;
plot(k,B_t(k,(np-1)*0.25));
ylabel('$V_{k}(t_p)|$','Interpreter','latex')
xlabel('k');
set(get(gca,'XLabel'),'FontSize',figure_FontSize);
set(get(gca,'YLabel'),'FontSize',figure_FontSize); 
set(gca,'fontsize',font_size);

subplot(212)
t=1:np;
plot(time(:),B_t(kmax,:));
ylabel('$V_{30}(t)$','Interpreter','latex');
xlabel('t');
set(get(gca,'XLabel'),'FontSize',figure_FontSize);
set(get(gca,'YLabel'),'FontSize',figure_FontSize); 
set(gca,'fontsize',font_size);


