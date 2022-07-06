%ME579_project_tracjectory test
%SI unit
function main
clear
clc
%======parameters
%flight control
g=9.81;m=0.468;l=0.225;k=2.980e-6;b=1.14e-7;I_M=3.357e-5;I_xx=4.856e-3;
I_yy=I_xx;I_zz=8.801e-3;A_x=0.25;A_y=0.25;A_z=0.25;

H_hover=2e2;H_drop=0.5e2;a1=5;a2=1;a3=5;a4=5;v1=10;v2=10;v3=20;v4=5;

%initial&final position
x0=0;y0=0;z0=0;x1=500;y1=200;z1=H_hover;z2=H_drop;
a=[a1;a2;a3;a4];v=[v1;v2;v3;v4];dt=0.001;

%======desired trajectory---dksi, ddksi, psi
[dksi,ddksi,psi,tt]=design(x0,y0,z0,x1,y1,z1,z2,a,v,dt);

%inplot(dksi,ddksi,psi,tt)
 
%======thrust & torques 
[thru,tao]=force(dksi,ddksi,psi,tt,m,A_x,A_y,A_z,I_xx,I_yy,I_zz,g,dt);
%thuplot(thru,tao,tt)

%======calculated tracjectory---omega, ksi, psi
[omega,ksi_c,eta_c]=forward(thru,tao,k,l,b,dt,tt);

%======plot
traplot(omega,ksi_c,eta_c,tt);



end

function [dksi,ddksi,psi,tt]=design(x0,y0,z0,x1,y1,z1,z2,a,v,dt)
b=0.5;c=2;tf=2*4*b+c+c/2;
ti=[b,3*b,4*b,4*b+c,5*b+c,7*b+c,8*b+c,tf];

a=1;
tt=[0:dt:tf];
nstep=(tf)/dt+1;
f=zeros(nstep,1);

for i=1:nstep
    if(tt(i)<ti(1))
            f(i)= a*sin(1/b*pi*tt(i));
    elseif(tt(i)>=ti(1)&tt(i)<ti(2))
            f(i)=-a*sin(1/2/b*pi*tt(i)-pi/2);
    elseif(tt(i)>=ti(2)&tt(i)<ti(3))
            f(i)=a*sin(1/b*pi*tt(i)-3*pi);
    elseif(tt(i)>=ti(3)&tt(i)<ti(4))
            f(i)=0;
    elseif(tt(i)>=ti(4)&tt(i)<ti(5))
            f(i)=-a*sin(1/b*pi*(tt(i)-c-4*b));
    elseif(tt(i)>=ti(5)&tt(i)<ti(6))
            f(i)=a*sin(1/2/b*pi*(tt(i)-c-4*b)-pi/2);
    elseif(tt(i)>=ti(6)&tt(i)<ti(7))
            f(i)=-a*sin(1/b*pi*(tt(i)-c-4*b)-3*pi);
    else
            f(i)=0;
    end
end
Jounce=f;
Jerk=intf(Jounce,tt);
Acce=intf(Jerk,tt);
Velo=intf(Acce,tt);
Posi=intf(Velo,tt);


% figure(1)
% plot(tt,Jounce,'r')
% figure(2)
% plot(tt,Jerk,'r')
% figure(3)
% plot(tt,Acce,'r')
% figure(4)
% plot(tt,Velo,'r')
% figure(5)
% plot(tt,Posi,'r')



% dksi=[zeros(1,nstep);Velo'*cos(pi/4);Velo'*cos(pi/4)];
% ddksi=[zeros(1,nstep);Acce'*sin(pi/4);Acce'*sin(pi/4)];

dksi=[Velo';zeros(1,nstep);zeros(1,nstep)];
ddksi=[Acce';zeros(1,nstep);zeros(1,nstep)];

% dksi=[zeros(1,nstep);Velo';zeros(1,nstep)];
% ddksi=[zeros(1,nstep);Acce';zeros(1,nstep)];
psi=zeros(nstep,1);

end
function [thru,tao]=force(dksi,ddksi,psi,tt,m,Ax,Ay,Az,I_xx,I_yy,I_zz,g,dt)
nstep=length(tt);
for i=1:nstep
    d_x=ddksi(1,i)+Ax*dksi(1,i)/m;
    d_y=ddksi(2,i)+Ay*dksi(2,i)/m;
    d_z=ddksi(3,i)+Az*dksi(3,i)/m;
    spsi=sin(psi(i));
    cpsi=cos(psi(i));
    phi(i)=asin((d_x*spsi-d_y*cpsi)/sqrt(d_x^2+d_y^2+(d_z+g)^2));
    theta(i)=atan((d_x*cpsi+d_y*spsi)/(d_z+g));
    
    cphi=cos(phi(i));
    sphi=sin(phi(i));
    ctheta=cos(theta(i));
    stheta=sin(theta(i));
    
    thru(1,i)=m*(d_x*(stheta*cpsi*cphi+spsi*sphi)+d_y*(stheta*spsi*cphi...
        -cpsi*sphi)+(d_z+g)*ctheta*cphi);
end

eta=[phi;theta;psi'];
%deta
deta=zeros(3,nstep);
size(gradient(phi))
    deta(1,:)=gradient(phi)/dt;
    deta(2,:)=gradient(theta)/dt;
    deta(3,:)=gradient(psi)/dt;

%ddeta
ddeta=zeros(3,nstep);

    ddeta(1,:)=gradient(deta(1,:))/dt;
    ddeta(2,:)=gradient(deta(2,:))/dt;
    ddeta(3,:)=gradient(deta(3,:))/dt;

    
%     figure(7)
%     plot(tt,eta(1,:),'r')
%     hold on
%     plot(tt,eta(2,:),'k')
%     plot(tt,eta(3,:),'b')
%     legend('\phi','\theta','\psi')
% %     
%     figure(8)
%     plot(tt,deta(1,:),'r')
%     hold on
%     plot(tt,deta(2,:),'k')
%     plot(tt,deta(3,:),'b')
%     
%     figure(9)
%     plot(tt,ddeta(1,:),'r')
%     hold on
%     plot(tt,ddeta(2,:),'k')
%     plot(tt,ddeta(3,:),'b')
    
    
    [J,Ceta]=JC(eta,deta,ddeta,I_xx,I_yy,I_zz);
    
    tao=J*ddeta+Ceta*deta;
    
%          figure(9)
%      plot(tt,tao(1,:),'r')
%      hold on
%      plot(tt,tao(2,:),'k')
%      plot(tt,tao(3,:),'b')
%      plot(tt(1:100:nstep),I_xx*ddeta(1,1:100:nstep),'o')
%      legend('\tau_1','\tau_2','\tau_3')
    
end
function [omega,ksi,eta]=forward(thru,tao,k,l,b,dt,tt)
%==========================================================================
g=9.81;m=0.468;I_M=3.357e-5;I_xx=4.856e-3;
I_yy=I_xx;I_zz=8.801e-3;A_x=0.25;A_y=0.25;A_z=0.25;

R=@(eta) [cos(eta(3))*cos(eta(2)),cos(eta(3))*sin(eta(2))*sin(eta(1))-sin(eta(3))*...
  cos(eta(1)),cos(eta(3))*sin(eta(2))*cos(eta(1))+sin(eta(3))*sin(eta(1));sin(eta(3))...
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
TB=@(omega) k*sum(omega.^2);
RTB=@(eta)[cos(eta(3))*sin(eta(2))*cos(eta(1))+sin(eta(3))*sin(eta(1));...
    sin(eta(3))*sin(eta(2))*cos(eta(1))-cos(eta(3))*sin(eta(1));...
    cos(eta(2))*cos(eta(1))];

tau_B=@(omega,domega)[l*k*(-omega(2)^2+omega(4)^2);l*k*(-omega(1)^2+omega(3)^2);...
    b*(omega(1)^2-omega(2)^2+omega(3)^2-omega(4)^2)+I_M*(domega(1)-domega(2)+...
    domega(3)-domega(4))]; %torque

ddksi=@(eta,omega,dksi)(-g*[0;0;1]+RTB(eta)*TB(omega)/m-1/m*diag([A_x,A_y,A_z])*dksi);
ddeta=@(eta,deta,omega,domega) inv(J(eta))*(tau_B(omega,domega)-C(eta,deta)*deta);
%==========================================================================

nstep=length(thru);
%omega
omega(1,:)=(thru(1,:)/4/k-tao(2,:)/2/k/l-tao(3,:)/4/b).^(1/2); 
omega(2,:)=(thru(1,:)/4/k-tao(1,:)/2/k/l+tao(3,:)/4/b).^(1/2);
omega(3,:)=(thru(1,:)/4/k+tao(2,:)/2/k/l-tao(3,:)/4/b).^(1/2);
omega(4,:)=(thru(1,:)/4/k+tao(1,:)/2/k/l+tao(3,:)/4/b).^(1/2);

%domega
domega=zeros(4,nstep);
for i=1:4
   domega(i,:)=gradient(omega(i,:))/dt; 
end


 eta_i=zeros(3,1);
 deta_i=zeros(3,1);
 omega_i=zeros(4,1);
 domega_i=zeros(4,1);
 ksi_i=zeros(3,1);
 dksi_i=zeros(3,1);
 ii=0;
 
[eta,ksi,deta,dksi,ddksii,RTBi]=propogate(nstep,eta_i,deta_i,omega_i,domega_i,ksi_i,dksi_i,ddeta,ddksi,RTB,dt,omega,domega,ii);

%  figure(20)
%  plot(tt(1:100:nstep),ddksii(1,1:100:nstep),'r*--')
%  hold on
%  plot(tt(1:100:nstep),ddksii(2,1:100:nstep),'ko--')
%  plot(tt(1:100:nstep),ddksii(3,1:100:nstep),'b.--')
%  legend('ddx','ddy','ddz')
%  
%  figure(21)
%  plot(tt(1:100:nstep),RTBi(1,1:100:nstep),'r*--')
%  hold on
%  plot(tt(1:100:nstep),RTBi(2,1:100:nstep),'ko--')
%  plot(tt(1:100:nstep),RTBi(3,1:100:nstep),'b.--')
%  legend('RTB1','RTB2','RTB3')
end
function traplot(omega,ksi,eta,tt)


size=25;
lag=10;
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
function thuplot(thru,tao,tt)
figure(4)
plot(tt,thru,'r-')
figure(5)
plot(tt,tao(1,:),'r-')
hold on
plot(tt,tao(2,:),'k-')
plot(tt,tao(3,:),'b-')
legend('\tau_1','\tau_2','\tau_3')
end
function [eta,ksi,deta,dksi,ddksii,RTBi]=propogate(nstep,eta_i,deta_i,omega_i,domega_i,ksi_i,dksi_i,ddeta,ddksi,RTB,dt,omega,domega,ii)
ksi(:,1)=ksi_i;
dksi(:,1)=dksi_i;
eta(:,1)=eta_i;%
deta(:,1)=eta_i;%
ddksii(:,1)=[0;0;0];
RTBi(:,1)=[0;0;1];
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
    ddksii(:,i)=ddksi_i;
    RTBi(:,i)=RTB(eta_i);
   if(isnan(eta_i)|isnan(ksi_i))
      disp('not converge for i=')
       i
      disp('program stop')
       break
   end
    
end
end
function inf=intf(f,tt)
nstep=length(f);
inf=zeros(nstep,1);
for i=2:nstep
    tti=tt(1:i);
    fi=f(1:i);
    inf(i)=trapz(tti,fi);
end
% for i=1:nstep
%    inf(i)=f(i)*dt; 
% end
end

