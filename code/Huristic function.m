%Test
function main
clear
clc
syms a b t c s positive
%assume(a,'positive');assume(b,'positive');assume(a,'positive');
y=a+b;
f1=a*sin(1/b*pi*t);
f2=-a*sin(1/2/b*pi*t-pi/2);
f3=a*sin(1/b*pi*t-3*pi);
f0=0*t;
f4=-a*sin(1/b*pi*(t-c-4*b));
f5=a*sin(1/2/b*pi*(t-c-4*b)-pi/2);
f6=-a*sin(1/b*pi*(t-c-4*b)-3*pi);
f7=0;
ti=[0,b,3*b,4*b,4*b+c,5*b+c,7*b+c,8*b+c,8*b+c+c/2];
Jounce=piecewise(t>=0&t<b,f1,t>=b&t<3*b,f2,t>=3*b&t<4*b,f3,...
    t>=4*b&t<(4*b+c),f0,t>=(4*b+c)&t<(5*b+c),f4,t>=(5*b+c)&...
    t<(7*b+c),f5,t>=(7*b+c)&t<=(8*b+c),f6,t>=(8*b+c)&t<=(8*b+c+c/2),f7);

f=[f1,f2,f3,f0,f4,f5,f6,f7];
f=intf(f,a,b,c,ti,t);
Jerk=piecewise(t<ti(2),f(1),t>=ti(2)&t<ti(3),f(2),t>=ti(3)&t<ti(4),f(3),...
     t>=ti(4)&t<ti(5),f(4),t>=ti(5)&t<ti(6),f(5),t>=ti(6)&t<ti(7),f(6)...
     ,t>=ti(7)&t<ti(8),f(7),t>=ti(8)&t<ti(9),f(8));
 
f=intf(f,a,b,c,ti,t);
Acce=piecewise(t<ti(2),f(1),t>=ti(2)&t<ti(3),f(2),t>=ti(3)&t<ti(4),f(3),...
     t>=ti(4)&t<ti(5),f(4),t>=ti(5)&t<ti(6),f(5),t>=ti(6)&t<ti(7),f(6)...
     ,t>=ti(7)&t<ti(8),f(7),t>=ti(8)&t<ti(9),f(8));
%  
f=intf(f,a,b,c,ti,t);
Velo=piecewise(t<ti(2),f(1),t>=ti(2)&t<ti(3),f(2),t>=ti(3)&t<ti(4),f(3),...
     t>=ti(4)&t<ti(5),f(4),t>=ti(5)&t<ti(6),f(5),t>=ti(6)&t<ti(7),f(6)...
     ,t>=ti(7)&t<ti(8),f(7),t>=ti(8)&t<ti(9),f(8));
%
f=intf(f,a,b,c,ti,t);
posi=piecewise(t<ti(2),f(1),t>=ti(2)&t<ti(3),f(2),t>=ti(3)&t<ti(4),f(3),...
     t>=ti(4)&t<ti(5),f(4),t>=ti(5)&t<ti(6),f(5),t>=ti(6)&t<ti(7),f(6)...
     ,t>=ti(7)&t<=ti(8),f(7),t>=ti(8)&t<=ti(9),f(8));
 
 length=simplify(subs(posi,t,ti(9)))
% 
 aa=solve(simplify(subs(posi,t,ti(9)))==s,a);

%evalute
s=10;b=0.5;c=2;dt=1e-2;t=[0:dt:subs(ti(9))];a=subs(aa);

Jounce=subs(Jounce);
Jerk=subs(Jerk);
Acce=subs(Acce);
Velo=subs(Velo);
posi=subs(posi);

 figure(1)
 plot(t,Jounce,'r-')
 figure(2)
 plot(t,Jerk,'r-')
 figure(3)
 plot(t,Acce,'r-')
 figure(4)
 plot(t,Velo,'r-')
 figure(5)
 plot(t,posi,'r-')
end
function f=intf(g,a,b,c,ti,t)
%
intg=int(g,t);
%
temp=0;
for i=1:8
   intg0(i)=subs(intg(i),t,ti(i));
   intg1(i)=subs(intg(i),t,ti(i+1));
   intgi(i)=intg1(i)-intg0(i);
   temp=temp+intgi(i);
   intgsum(i+1)=temp;
end
intgsum(1)=0;
f=intg-intg0+intgsum(1:8);
end

