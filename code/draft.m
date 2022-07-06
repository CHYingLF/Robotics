%ME579_project_design
%SI unit
function main
clear
clc
%======parameters
%flight control
g=9.81;m=0.468;l=0.225;k=2.980e-6;b=1.14e-7;I_M=3.357e-5;I_xx=4.856e-3;
I_yy=I_xx;I_zz=8.801e-3;A_x=0.0;A_y=0.0;A_z=0.0;

H_hover=2e2;H_drop=0.5e2;a1=5;a2=1;a3=5;a4=5;v1=10;v2=10;v3=20;v4=5;

%initial&final position
x0=0;y0=0;z0=0;x1=500;y1=200;z1=H_hover;z2=H_drop;
a=[a1;a2;a3;a4];v=[v1;v2;v3;v4];dt=0.001;

%======desired trajectory---dksi, ddksi, psi
[dksi,ddksi,psi,tt]=design(x0,y0,z0,x1,y1,z1,z2,a,v,dt);

inplot(dksi,ddksi,psi,tt)
 
%======thrust & torques 
[thru,tao]=force(dksi,ddksi,psi,tt,m,A_x,A_y,A_z,I_xx,I_yy,I_zz,g,dt);
figure(4)
plot(tt,thru,'r-')
figure(5)
plot(tt,tao(1,:),'r-')
hold on
plot(tt,tao(2,:),'k-')
plot(tt,tao(3,:),'b-')
legend('\tau_1','\tau_2','\tau_3')

%======calculated tracjectory---omega, ksi, psi
%[omega,ksi_c,eta_c]=forward(thru,tao,k,l,b,dt);

%======plot
%traplot(omega,ksi_c,eta_c,tt);



end

function [dksi,ddksi,psi,tt]=design(x0,y0,z0,x1,y1,z1,z2,a,v,dt)
dksi=[];ddksi=[];psi=[];tt=[];
%---------
%ascending
ii=1;
h(ii)=z1-z0;
T(ii)=h/v(1);
t1=v(ii)/a(ii);
ti=[0:dt:T];
nstep=length(ti);
dksi_t(1,1:nstep)=0;
dksi_t(2,1:nstep)=0;
ddksi_t(1,1:nstep)=0;
ddksi_t(2,1:nstep)=0;
psi_t(1:nstep)=0;
for i=1:nstep
   if(ti(i)<t1)
       dksi_t(3,i)=a(ii)*ti(i);
       ddksi_t(3,i)=a(ii);
   elseif(ti(i)>(T-t1))
       dksi_t(3,i)=h(ii)/v(ii)*a(ii)-a(ii)*ti(i);
       ddksi_t(3,i)=-a(ii);
   else
       dksi_t(3,i)=v(ii);
       ddksi_t(3,i)=0;
   end
end

dksi=[dksi,dksi_t];
ddksi=[ddksi,ddksi_t];
psi=[psi,psi_t];
tt=[tt,ti];

%---------
%yawing
ii=2;
h=sqrt((x1-x0)^2+(y1-y0)^2);
alpha=asin((y1-y0)/h);
T(ii)=alpha/a(ii);
ti=[0:dt:T(ii)];
nstep=length(ti);
dksi_t=[]
dksi_t=[];
ddksi_t=[];
psi_t=zeros(1,nstep);
for i=1:nstep
       dksi_t(1:3,i)=0;
       ddksi_t(1:3,i)=0;
       psi_t(1,i)=ti(i)*a(ii);
end

dksi=[dksi,dksi_t];
ddksi=[ddksi,ddksi_t];
psi=[psi,psi_t];
tt=[tt,ti+T(ii-1)];

%---------
%hovering
ii=3;
h=sqrt((x1-x0)^2+(y1-y0)^2);
alpha=asin((y1-y0)/h);
v(ii)*sin(alpha)
v(ii)*cos(alpha)
T(ii)=h/v(ii);
t1=v(ii)/a(ii);
ti=[0:dt:T(ii)];
nstep=length(ti);
dksi_t=[]
dksi_t=[];
ddksi_t=[];
psi_t=[];
for i=1:nstep
    psi_t(i)=alpha;
   if(ti(i)<t1)
       dksi_t(1,i)=a(ii)*ti(i)*cos(alpha);
       dksi_t(2,i)=a(ii)*ti(i)*sin(alpha);
       dksi_t(3,i)=0;
       ddksi_t(1,i)=a(ii)*cos(alpha);
       ddksi_t(2,i)=a(ii)*sin(alpha);
       ddksi_t(3,i)=0;
   elseif(ti(i)>(T(ii)-t1))
       dksi_t(1,i)=(h/v(ii)*a(ii)-a(ii)*ti(i))*cos(alpha);
       dksi_t(2,i)=(h/v(ii)*a(ii)-a(ii)*ti(i))*sin(alpha);
       dksi_t(3,i)=0;
       ddksi_t(1,i)=-a(ii)*cos(alpha);
       ddksi_t(2,i)=-a(ii)*sin(alpha);
       ddksi_t(3,i)=0;
   else
       dksi_t(1,i)=v(ii)*cos(alpha);
       dksi_t(2,i)=v(ii)*sin(alpha);
       dksi_t(3,i)=0;
       ddksi_t(1,i)=0;
       ddksi_t(2,i)=0;
       ddksi_t(3,i)=0;
   end
end
dksi=[dksi,dksi_t];
ddksi=[ddksi,ddksi_t];
psi=[psi,psi_t];
tt=[tt,ti+sum(T(1:ii-1))];

%---------
%descending
ii=4;
h=z1-z2;
T(ii)=h/v(ii);
t1=v(ii)/a(ii);
ti=[0:dt:T(ii)];
nstep=length(ti);
dksi_t=[]
dksi_t=[];
ddksi_t=[];
psi_t=[];
for i=1:nstep
    psi_t(i)=alpha;
   if(ti(i)<t1)
       dksi_t(1,i)=0;
       dksi_t(2,i)=0;
       dksi_t(3,i)=-a(ii)*ti(i);
       ddksi_t(1,i)=0;
       ddksi_t(2,i)=0;
       ddksi_t(3,i)=-a(ii);
   elseif(ti(i)>(T(ii)-t1))
       dksi_t(1,i)=0;
       dksi_t(2,i)=0;
       dksi_t(3,i)=a(ii)*ti(i)-h*a(ii)/v(ii);
       ddksi_t(1,i)=0;
       ddksi_t(2,i)=0;
       ddksi_t(3,i)=a(ii);
   else
       dksi_t(1,i)=0;
       dksi_t(2,i)=0;
       dksi_t(3,i)=-v(ii);
       ddksi_t(1,i)=0;
       ddksi_t(2,i)=0;
       ddksi_t(3,i)=0;
   end
end
dksi=[dksi,dksi_t];
ddksi=[ddksi,ddksi_t];
psi=[psi,psi_t];
tt=[tt,ti+sum(T(1:ii-1))];

end
function [thru,tao]=force(dksi,ddksi,psi,tt,m,Ax,Ay,Az,I_xx,I_yy,I_zz,g,dt)
nstep=length(tt);
for i=1:nstep
    d_x=ddksi(1,i)+Ax*dksi(1,i)/m;
    d_y=ddksi(2,i)+Ay*dksi(2,i)/m;
    d_z=ddksi(3,i)+Az*dksi(3,i)/m;
    spsi=sin(psi(i));
    cpsi=cos(psi(i));
    phi(i)=asin((d_x*spsi-d_y*cpsi)/(d_x^2+d_y^2+(d_z+g)^2));
    theta(i)=atan((d_x*cpsi+d_y*spsi)/(d_z+g));
    
    cphi=cos(phi(i));
    sphi=sin(phi(i));
    ctheta=cos(theta(i));
    stheta=sin(theta(i));
    
    thru(1,i)=m*(d_x*(stheta*cpsi*cphi+spsi*sphi)+d_y*(stheta*spsi*cphi...
        -cpsi*sphi)+(d_z+g)*ctheta*cphi);
end
eta=[phi;theta;psi];
%deta
deta=zeros(3,nstep);
for i=2:nstep-1
    deta(1,i)=(phi(i+1)-phi(i-1))/2/dt;
    deta(2,i)=(theta(i+1)-theta(i-1))/2/dt;
    deta(3,i)=(psi(i+1)-psi(i-1))/2/dt;
end
    deta(1,nstep)=(phi(nstep)-phi(nstep-1))/dt;
    deta(2,nstep)=(theta(nstep)-theta(nstep-1))/dt;
    deta(3,nstep)=(psi(nstep)-psi(nstep-1))/dt;
%ddeta
ddeta=zeros(3,nstep);
for i=2:nstep-1
    ddeta(1,i)=(deta(1,i+1)-deta(1,i-1))/2/dt;
    ddeta(2,i)=(deta(2,i+1)-deta(2,i-1))/2/dt;
    ddeta(3,i)=(deta(3,i+1)-deta(3,i-1))/2/dt;
end
    ddeta(1,nstep)=(deta(1,nstep)-deta(1,nstep-1))/dt;
    ddeta(2,nstep)=(deta(2,nstep)-deta(2,nstep-1))/dt;
    ddeta(3,nstep)=(deta(3,nstep)-deta(3,nstep-1))/dt;
    
    figure(7)
    plot(tt,eta(1,:),'r')
    hold on
    plot(tt,eta(2,:),'k')
    plot(tt,eta(3,:),'b')
    
    figure(8)
    plot(tt,deta(1,:),'r')
    hold on
    plot(tt,deta(2,:),'k')
    plot(tt,deta(3,:),'b')
    
    figure(9)
    plot(tt,ddeta(1,:),'r')
    hold on
    plot(tt,ddeta(2,:),'k')
    plot(tt,ddeta(3,:),'b')
    
    
    [J,Ceta]=JC(eta,deta,ddeta,I_xx,I_yy,I_zz);
    
    tao=J*ddeta+Ceta*deta;
    
end
function [omega,ksi,eta]=forward(thru,tao,k,l,b,dt)
%==========================================================================
g=9.81;m=0.468;I_M=3.357e-5;I_xx=4.856e-3;
I_yy=I_xx;I_zz=8.801e-3;A_x=0.25;A_y=0.25;A_z=0.25;

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
%==========================================================================

nstep=length(thru);
%omega
omega(1,:)=(thru(1,:)/4/k-tao(2,:)/2/k/l-tao(3,:)/4/b).^(1/2); 
omega(2,:)=(thru(1,:)/4/k-tao(1,:)/2/k/l+tao(3,:)/4/b).^(1/2);
omega(3,:)=(thru(1,:)/4/k+tao(2,:)/2/k/l-tao(3,:)/4/b).^(1/2);
omega(4,:)=(thru(1,:)/4/k+tao(1,:)/2/k/l+tao(3,:)/4/b).^(1/2);
for i=1:nstep
j=1;
if(isreal(omega(j,i))==0)
    i
   omega(j,i) 
end
end
%domega
domega=zeros(4,nstep);
% for i=2:nstep-1
%  domega(:,i)=(omega(:,i+1)-omega(:,i-1))/2/dt;
% end
%  domega(:,nstep)=(omega(:,nstep)-omega(:,nstep-1))/dt;

 eta_i=zeros(3,1);
 deta_i=zeros(3,1);
 omega_i=zeros(4,1);
 domega_i=zeros(4,1);
 ksi_i=zeros(3,1);
 dksi_i=zeros(3,1);
 ii=0;
 
 eta=0;
 ksi=0;
%[eta,ksi,deta,dksi]=propogate(nstep,eta_i,deta_i,omega_i,domega_i,ksi_i,dksi_i,ddeta,ddksi,dt,omega,domega,ii);


end
function traplot(omega,ksi_c,eta_c,tt)

plot(tt,omega(1,:),'-r')
hold on
plot(tt,omega(2,:),'-k')
plot(tt,omega(3,:),'-b')
plot(tt,omega(4,:),'-m')
legend('omega_1','omega_2','omega_3','omega_4')
end
function [J,Ceta]=JC(eta,deta,ddeta,I_xx,I_yy,I_zz)
W_eta=[1,0,-sin(eta(2));0,cos(eta(1)),cos(eta(2))*sin(eta(1));...
    0,-sin(eta(1)),cos(eta(2))*cos(eta(1))];

I=diag([I_xx,I_yy,I_zz]);

J= W_eta'*I*W_eta;

Ceta=[0,(I_yy-I_zz)*(deta(2)*cos(eta(1))*sin(eta(1))+deta(3)*...
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
end
function inplot(dksi,ddksi,psi,tt)
le=length(tt);
lag=1;
figure(1)
plot(tt(1:lag:le),dksi(1,1:lag:le),'-ro');
hold on
plot(tt(1:lag:le),dksi(2,1:lag:le),'-g*');
plot(tt,dksi(3,:),'-b');
legend('vx','vy','vz')
figure(2)
plot(tt,ddksi(1,:),'-ro');
hold on
plot(tt,ddksi(2,:),'-g*');
plot(tt,ddksi(3,:),'-b');
legend('ax','ay','az')
figure(3)
plot(tt,psi,'-b');
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
      disp('program stop')
       break
   end
    
end
end


