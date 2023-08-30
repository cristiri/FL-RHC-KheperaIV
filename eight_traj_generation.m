%% TRAJECTORY PARAMETERS
% Initial conditions
%scaling factor of the trajectory
%Initial conditions

%% TRAJECTORY PARAMETERS
%scaling factor of the trajectory
eta=0.6;
alpha=3.4;
k=0:Ts:2*pi*alpha*2+0.85;
xr=eta*sin(k/alpha);
yr=eta*sin(k/(2*alpha));


%Velocity trajectory
xpr=eta*cos(k/alpha)*(1/alpha);
ypr=eta*cos(k/(2*alpha))*(1/(2*alpha));

%Acceleration trajectory
xppr=-eta*sin(k/alpha)*(1/alpha)*(1/alpha);
yppr=-eta*sin(k/(2*alpha))*(1/(2*alpha))*(1/(2*alpha));

% Orientation reference
thetar=atan2(ypr,xpr);
thetar_diff=diff(thetar);

for i=1:length(thetar_diff)
   if thetar_diff(i)<-6 
       i1=i+1;
   elseif thetar_diff(i)>6 
       i2=i;
   end
end


thetar(i1:i2)=thetar(i1:i2)+2*pi; %Tc=0.15
% thetar(159:472)=thetar(159:472)+2*pi; %Tc=0.2

x0=0.6; y0=0; theta0=pi;

r=0.0210;
d=0.10470;
M=[[r/2 r/2];[r/d -r/d]]; %Transformation Matrix
M_inv=inv(M); % Inverse Transformation Matrix
desired_max_speed=300;
max_speed_rad=0.813/0.0210;
desired_max_speed_rad=desired_max_speed*max_speed_rad/1200;


H_square=[-1 0;0 -1;1 0;0 1];
H=H_square*M_inv;
H=H/desired_max_speed_rad;
g=ones(4,1);

poly=Polyhedron(H,g);


vr=sqrt(xpr.^2+ypr.^2);
wr=(yppr.*xpr-xppr.*ypr)./(xpr.^2+ypr.^2);
figure
plot(poly)
hold on
plot(vr,wr)
