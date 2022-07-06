%Test
function main
clear
clc
k=2.980e-6;
B=620.6108;
g=9.81;
pi=3.14;
m=0.468;
T=0.5;
w=2*pi/T;

a=k/16;
b=4*k*B;
c=1/8*(k*B^2-m*g)-0.1*m;

A=(-b+sqrt(b^2-4*a*c))/2/a;

t=[0:0.01:0.5];
w=A*sin(w*t)+B;
%plot(t,w,'-r');

w=zeros(4,2);
w(:,:)=1;


cpl=2300;
cps=1800;
Tm=35.6;
h_fus=239e3;
deltaT=3;

T1=30;
T2=40;
N=100;
dT=(T2-T1)/N
T=T1:dT:T2;


for i=1:N+1
    if(T(i)<Tm-deltaT/2.0)
        CpT(i)=cps;
    elseif(T(i)>Tm+deltaT/2.0)
        CpT(i)=cpl;
    else
        GDT=exp((T(i)-Tm).^2/(deltaT/4.0)^2)/sqrt(pi*(deltaT/4.0)^2);
        phiT=phi(T(i),deltaT,Tm);
        CpT(i)=cps+phiT*(cpl-cps)+h_fus*GDT;
    end
end

%plot(T,CpT,'*-')
xlabel('T(celsius)')
ylabel('Cp')


h_fus
DT=exp((3).^2/(deltaT/4.0)^2)/sqrt(pi*(deltaT/4.0)^2)
G=h_fus*DT
end


function phiT=phi(T,deltaT,Tm)
if(T<Tm-deltaT/2) 
    phiT=0;
elseif(T>Tm+deltaT/2)
    phiT=1.0;
else
    phiT=(T-(Tm-deltaT/2))/deltaT;
end
end
