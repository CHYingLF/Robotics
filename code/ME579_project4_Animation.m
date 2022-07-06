%ME579_project_Animation
%SI unit
clc
clear
ksi=csvread('ksi.csv');
L=size(ksi);
b=0.5;c=2;tf=2*4*b+c+c;dt=0.001;
H_hover=50;H_drop=0.1e2;
x0=0;y0=0;z0=0;x1=10;y1=10;z1=H_hover;z2=H_drop;
tt=[1:L(2)]*dt;

% plot(tt,ksi(1,:),'-r')
% hold on
% plot(tt,ksi(2,:),'--k')
% plot(tt,ksi(3,:),'-.b')
% legend('x','y','z')
% axis([0 35 0 50]);
%figure(2)
t = 0:pi/50:10*pi;
st = sin(t);
ct = cos(t);
%plot3(0,0,0)


h = figure;
axis tight manual % this ensures that getframe() returns a consistent size
filename = 'trajectory.gif';
d=400;
for k=1:d:L(2)
   cla
   %calculated trajectory
   %hold on
   %links and ball
    plot3([ksi(1,k)-1,ksi(1,k)+1],[ksi(2,k),ksi(2,k)],[ksi(3,k),ksi(3,k)],'r-')
    hold on
    plot3([ksi(1,k),ksi(1,k)],[ksi(2,k)-1,ksi(2,k)+1],[ksi(3,k),ksi(3,k)],'b-')
%    plot([x1(1),x2(1)],[x1(2),x2(2)],'g-')
%    plot([x2(1),x3(1)],[x2(2),x3(2)],'b-')
%    plot(L3/2,yb(1,k),'ro')
   plot3(ksi(1,k),ksi(2,k),ksi(3,k),'ok','MarkerSize',2)
   
   %line
   for i=1:d:k-d
      plot3([ksi(1,i),ksi(1,i+d)],[ksi(2,i),ksi(2,i+d)],[ksi(3,i),ksi(3,i+d)],'-.k') 
       
   end
   
   
   plot3(x0,y0,z0,'*')
   plot3(x1,y1,z2,'*')
   text(x0+0.5,y0,z0-1,'P_{initial}');
   text(x1+0.5,y1,z2-1,'P_{final}');
   
   axis([-2  12  -2  12  0  55])
   xlabel('X(m)')
   ylabel('Y(m)')
   zlabel('Z(m)')
   % Capture the plot as an image 
      frame = getframe(h); 
      im = frame2im(frame); 
      [imind,cm] = rgb2ind(im,256); 
      % Write to the GIF File 
      if k == 1 
          imwrite(imind,cm,filename,'gif', 'Loopcount',inf); 
      else 
          imwrite(imind,cm,filename,'gif','WriteMode','append'); 
      end 
   %axis([-2 3 -2 yb0+2*L1])
   pause(0.05)
end
filename
%set(gca, 'CameraPosition', [100 5000 2000]);