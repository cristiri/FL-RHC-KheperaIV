% Elliptic trajectory
alpha=3.8;
k=0:Ts:2*alpha*pi+Ts;
vxbar=0.55; vybar=0.55;
x0=vxbar; y0=vybar; theta0=pi;

xr=vxbar*cos(k/alpha);
yr=vybar*sin(k/alpha);

xpr=-1/alpha*vxbar*sin(k/alpha);
ypr= 1/alpha*vybar*cos(k/alpha);

xppr=-1/alpha^2*vxbar*cos(k/alpha);
yppr=-1/alpha^2*vybar*sin(k/alpha);

%Orientation reference
thetar=atan2(ypr,xpr);

thetar_diff=diff(thetar);

for i=1:length(thetar_diff)
   if thetar_diff(i)<-6 
       i1=i+1;
   end
end


thetar(i1:end)=thetar(i1:end)+2*pi; %Tc=0.15


r=0.0210;
d=0.10470;
M=[[r/2 r/2];[r/d -r/d]]; %Transformation Matrix
M_inv=inv(M); % Inverse Transformation Matrix
desired_max_speed=250;
max_speed_rad=0.813/0.0210;
desired_max_speed_rad=desired_max_speed*max_speed_rad/1200;


H_square=[-1 0;0 -1;1 0;0 1];
H=H_square*M_inv;
H=H/desired_max_speed_rad;
g=ones(4,1);

poly=Polyhedron(H,g);


vr=sqrt(xpr.^2+ypr.^2);
wr=(yppr.*xpr-xppr.*ypr)./(xpr.^2+ypr.^2);

% plot(poly)
plot(poly)
hold on

plot(vr,wr,'b--x')



