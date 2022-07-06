%Meta579 Projetact
function main
clc
clear
g=9.81;
m=0.468;
l=0.225;
k=2.980e-6;
b=1.14e-7;
I_M=3.357e-5;
I_xx=4.856e-3;
I_yy=I_xx;
I_zz=8.801e-3;
A_x=0.25;
A_y=0.25;
A_z=0.25;
deta=[1,1,1];
eta=[1,1,1];

R=@(eta) [cos(eta(3)*cos(eta(2))),cos(eta(3))*sin(eta(2))*sin(eta(1))-sin(eta(3)*...
  cos(eta(1))),cos(eta(3))*sin(eta(2))*cos(eta(1))+sin(eta(3))*sin(eta(1));sin(eta(3))...
  *cos(eta(2)),sin(eta(3))*sin(eta(2))*sin(eta(1))+cos(eta(3))*cos(eta(1)),sin(eta(3))...
  *sin(eta(2))*cos(eta(1))-cos(eta(3))*sin(eta(1));-sin(eta(2)),cos(eta(2))*sin(eta(1))...
  ,cos(eta(2))*cos(eta(1))]; %rotation matrix 

W_eta=@(eta)[1,0,-sin(eta(2));0,cos(eta(1)),cos(eta(2))*sin(eta(1));...
    0,-sin(eta(1)),cos(eta(2))*cos(eta(1))];

I=diag([I_xx,I_yy,I_zz]);

J=@(eta) W_eta(eta)'*I*W_eta(eta);

C=@(eta,deta)[0,(I_yy-I_zz)*(deta(2)*cos(eta(1))*sin(eta(1))+deta(3)*...
    sin(eta(1))^2*cos(eta(2)))+(I_zz-I_yy)*deta(3)*cos(eta(1))^2*...
    cos(eta(2))-I_xx*deta(3)*cos(eta(2)),(I_zz-I_yy)*deta(3)*cos(eta(1))*...
    sin(eta(1))*cos(eta(2))^2;(I_zz-I_yy)*(deta(2)*cos(eta(1))*sin(eta(1))...
    +deta(3)*sin(eta(1))^2*cos(eta(2)))+(I_yy-I_zz)*deta(3)*cos(eta(1))^2*...
    cos(eta(2))+I_xx*deta(3)*cos(eta(2)),(I_zz-I_yy)*deta(1)*cos(eta(1))*...
    sin(eta(1)),-I_xx*deta(3)*sin(eta(2))*cos(eta(2))+I_yy*deta(3)*...
    sin(eta(1))^2*sin(eta(2))*cos(eta(2))+I_zz*deta(3)*cos(eta(1))^2*...
    sin(eta(2))*cos(eta(2));(I_yy-I_zz)*deta(3)*cos(eta(2))^2*sin(eta(1))...
    *cos(eta(1))-I_xx*deta(2)*cos(eta(2)),(I_zz-I_yy)*(deta(2)*cos(eta(1))...
    *sin(eta(1))*sin(eta(2))+deta(1)*sin(eta(1))^2*cos(eta(2)))+(I_yy-I_zz)*...
    deta(1)*cos(eta(1))^2*cos(eta(2))+I_xx*deta(3)*sin(eta(2))*cos(eta(2))-...
    I_yy*deta(3)*sin(eta(1))^2*sin(eta(2))*cos(eta(2))-I_zz*deta(3)*...
    cos(eta(1))^2*sin(eta(2))*cos(eta(2)),(I_yy-I_zz)*deta(1)*cos(eta(1))*...
    sin(eta(1))*cos(eta(2))^2-I_yy*deta(2)*sin(eta(1))^2*cos(eta(2))*...
    sin(eta(2))-I_zz*deta(2)*cos(eta(1))^2*cos(eta(2))*sin(eta(2))+...
    I_xx*deta(2)*cos(eta(2))*sin(eta(2))]; %Coriolis term


T_B=@(omega)[0;0;k*sum(omega.^2)]; %force

tau_B=@(omega,domega)[l*k*(-omega(2)^2+omega(4)^2);l*k*(-omega(1)^2+omega(3)^2);...
    b*(omega(1)^2-omega(2)^2+omega(3)^2-omega(4)^2)+I_M*(domega(1)-domega(2)+...
    domega(3)-domega(4))]; %torque

ddksi=@(eta,omega,dksi)(-g*[0;0;1]+1/m*R(eta)*T_B(omega)-1/m*diag([A_x,A_y,A_z])*dksi);
ddeta=@(eta,deta,omega,domega) inv(J(eta))*(tau_B(omega,domega)-C(eta,deta)*deta);

%%Test1
T=0.5;
dt=0.0001;
nstep=T/dt+1;

%initial
ksi0=[0;0;0];
dksi0=[0;0;0];
eta0=[0;0;0];
deta0=[0;0;0];
ksi=zeros(3,nstep);
dksi=zeros(3,nstep);
eta=zeros(3,nstep);
deta=zeros(3,nstep);


%input
%1st, ascend
T1=0.5;
ti1=[0:dt:T1];
nstep1=T1/dt+1;
B=sqrt(m*g/k/4);
A=64.404129435620234;
wf1=2*pi/T1; %wave number
omega1=zeros(4,nstep1);
domega1=zeros(4,nstep1);
omega1(1,:)=A*sin(wf1*ti1)+B;%input angular velosity
omega1(2,:)=A*sin(wf1*ti1)+B;
omega1(3,:)=A*sin(wf1*ti1)+B;
omega1(4,:)=A*sin(wf1*ti1)+B;
domega1(1,:)=A*wf1*cos(wf1*ti1);
domega1(2,:)=A*wf1*cos(wf1*ti1);
domega1(3,:)=A*wf1*cos(wf1*ti1);
domega1(4,:)=A*wf1*cos(wf1*ti1);

%temp. value
ksi_i=ksi0;
dksi_i=dksi0;
eta_i(1:3,1)=eta0;
deta_i(1:3,1)=deta0;
omega_i(1:4,1)=omega1(1:4,1);
domega_i(1:4,1)=domega1(1:4,1);

[eta1,ksi1,deta1,dksi1]=propogate(nstep,eta_i,deta_i,omega_i,domega_i,ksi_i,dksi_i,ddeta,ddksi,dt,omega1,domega1,0);

%input
%2nd, roll
T2=0.5;
ti2=T1+[0:dt:T1];
nstep2=T1/dt+1;
B2=sqrt(m*g/k/4);
A2=650-B2;
wf2=2*pi/T2; %wave number
omega2=zeros(4,nstep2);
domega2=zeros(4,nstep2);
omega2(1,:)=B;%input angular velosity
omega2(2,:)=-A2*sin(wf2*ti2)+B2;
omega2(3,:)=B;
omega2(4,:)=A2*sin(wf2*ti2)+B2;
domega2(1,:)=0;
domega2(2,:)=-A2*wf1*cos(wf2*ti2);
domega2(3,:)=0;
domega2(4,:)=A2*wf1*cos(wf2*ti2);

%temp. value
ksi_i=ksi1(:,nstep1);
dksi_i=dksi1(:,nstep1);
eta_i(1:3,1)=eta1(:,nstep1);
deta_i(1:3,1)=deta1(:,nstep1);
omega_i(1:4,1)=omega2(1:4,1);
domega_i(1:4,1)=domega2(1:4,1);

[eta2,ksi2,deta2,dksi2]=propogate(nstep,eta_i,deta_i,omega_i,domega_i,ksi_i,dksi_i,ddeta,ddksi,dt,omega2,domega2,0);


%--------------
%input
%3rd, pitch
T3=0.5;
ti3=T1+T2+[0:dt:T3];
nstep3=T3/dt+1;
B3=sqrt(m*g/k/4);
A3=650-B3;
wf3=2*pi/T3; %wave number
omega3=zeros(4,nstep3);
domega3=zeros(4,nstep3);
omega3(1,:)=-A3*sin(wf3*ti3)+B2;%input angular velosity
omega3(2,:)=B3;
omega3(3,:)=A2*sin(wf3*ti3)+B2;
omega3(4,:)=B3;
domega3(1,:)=0;
domega3(2,:)=-A3*wf3*cos(wf3*ti3);
domega3(3,:)=0;
domega3(4,:)=A3*wf3*cos(wf3*ti3);

%temp. value
ksi_i=ksi2(:,nstep2);
dksi_i=dksi2(:,nstep2);
eta_i(1:3,1)=eta2(:,nstep2);
deta_i(1:3,1)=deta2(:,nstep2);
omega_i(1:4,1)=omega2(1:4,1);
domega_i(1:4,1)=domega2(1:4,1);

[eta3,ksi3,deta3,dksi3]=propogate(nstep,eta_i,deta_i,omega_i,domega_i,ksi_i,dksi_i,ddeta,ddksi,dt,omega3,domega3,0);

%--------------
%input
%4th, yaw
T4=0.5;
ti4=T1+T2+T3+[0:dt:T4];
nstep4=T4/dt+1;
B4=sqrt(m*g/k/4);
A4=650-B4;
wf4=2*pi/T4; %wave number
omega4=zeros(4,nstep4);
domega4=zeros(4,nstep4);
omega4(1,:)=A4*sin(wf4*ti4)+B4;%input angular velosity
omega4(2,:)=-A4*sin(wf4*ti4)+B4;
omega4(3,:)=A4*sin(wf4*ti4)+B4;
omega4(4,:)=-A4*sin(wf4*ti4)+B4;
domega4(1,:)=A4*wf4*cos(wf4*ti4);
domega4(2,:)=-A4*wf4*cos(wf4*ti4);
domega4(3,:)=A4*wf4*cos(wf4*ti4);
domega4(4,:)=-A4*wf4*cos(wf4*ti4);

%temp. value
ksi_i=ksi3(:,nstep3);
dksi_i=dksi3(:,nstep3);
eta_i(1:3,1)=eta3(:,nstep3);
deta_i(1:3,1)=deta3(:,nstep3);
omega_i(1:4,1)=omega3(1:4,1);
domega_i(1:4,1)=domega3(1:4,1);

[eta4,ksi4,deta4,dksi4]=propogate(nstep,eta_i,deta_i,omega_i,domega_i,ksi_i,dksi_i,ddeta,ddksi,dt,omega4,domega4,0);

eta=[eta1,eta2,eta3,eta4];
ksi=[ksi1,ksi2,ksi3,ksi4];
omega=[omega1,omega2,omega3,omega4];
%==========================================================================
ti=[ti1,ti2,ti3,ti4];
length(ti);
length(eta);

%omega
% figure(1)
% plot(ti,omega(1,:),'r-')
% hold on
% plot(ti,omega(2,:),'k--')
% plot(ti,omega(3,:),'b-.')
% plot(ti,omega(4,:),'m:','Linewidth',1)
% xlabel('t(s)')
% ylabel('Angular velosity(rad/s)')
% legend('\omega_1','\omega_2','\omega_3','\omega_4')
% axis([0 2 550 700]);
% set(gca,'Fontsize',18);
% 
% %position
% figure(2)
% plot(ti,ksi(1,:),'r-')
% hold on
% plot(ti,ksi(2,:),'k--')
% plot(ti,ksi(3,:),'b-.')
% xlabel('t(s)')
% ylabel('Position(m)')
% legend('x','y','z')
% axis([0 2 -2.5 1.5]);
% set(gca,'Fontsize',18);
% 
% %angle
% figure(3)
% plot(ti,eta(1,:)*180/pi,'r-')
% hold on
% plot(ti,eta(2,:)*180/pi,'k--')
% plot(ti,eta(3,:)*180/pi,'b-.')
% xlabel('t(s)')
% ylabel('Angles(degree)')
% legend('\phi','\theta','\psi')
% axis([0 2 -5 30]);
% set(gca,'Fontsize',18);
size=25;
tt=ti;
lag=100;
steps=1:lag:length(tt);
figure(100)
plot(tt(steps),omega(1,steps),'--*r')
hold on
plot(tt(steps),omega(2,steps),'--ok')
plot(tt(steps),omega(3,steps),'--.b')
plot(tt(steps),omega(4,steps),'-m')
xlabel('t(s)')
ylabel('Angular velocity(rad/s)')
set(gca,'Fontsize',size);
legend('\omega_1','\omega_2','\omega_3','\omega_4')

figure(101)
plot(tt(steps),ksi(1,steps),'-or')
hold on
plot(tt(steps),ksi(2,steps),'--*k')
plot(tt(steps),ksi(3,steps),'.-.b')
xlabel('t(s)')
ylabel('position(m)')
set(gca,'Fontsize',size);
legend('x','y','z')

figure(102)
plot(tt(steps),eta(1,steps)*180/pi,'-or')
hold on
plot(tt(steps),eta(2,steps)*180/pi,'--*k')
plot(tt(steps),eta(3,steps)*180/pi,'.-.b')
xlabel('t(s)')
ylabel('Angle(degree)')
set(gca,'Fontsize',size);
legend('\phi','\theta','\psi')

format long
end

function [eta,ksi,deta,dksi]=propogate(nstep,eta_i,deta_i,omega_i,domega_i,ksi_i,dksi_i,ddeta,ddksi,dt,omega,domega,ii)
ksi(:,1)=ksi_i;
dksi(:,1)=dksi_i;
eta(:,1)=eta_i;%
deta(:,1)=eta_i;%

for i=2:nstep
    %acceleration
    ddeta_i=ddeta(eta_i,deta_i,omega_i,domega_i);
    ddksi_i=ddksi(eta_i,omega_i,dksi_i);
    
    %velosity
    deta_i=deta_i+ddeta_i*dt;
    dksi_i=dksi_i+ddksi_i*dt;
    
    %position
    eta_i=eta_i+deta_i*dt;
    ksi_i=ksi_i+dksi_i*dt;
    omega_i=omega(:,i);
    domega_i=domega(:,i);
    
    eta(:,i)=eta_i;%
    ksi(:,i)=ksi_i;%
    dksi(:,i)=dksi_i;
    deta(:,i)=deta_i;
    
   if(isnan(eta_i)|isnan(ksi_i))
      disp('not converge for i=')
       i
   end
    
end
end

