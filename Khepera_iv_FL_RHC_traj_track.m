clear; close all;

Ts=0.15;

%Generation of xr, yr, tehtar, xpr, ypr, xppr, yppr defining the reference
%trajectory
% circular_traj_generation
eight_traj_generation
% square_traj_generation
% Z_traj_generation

%Defining control parameters
port='/dev/rfcomm0';
max_wheels_speed=250;
Ru=0.0001*eye(2);
Qx=1000*eye(2);
b=0.1;
param=STTT_control_parameters(Ts,b,Qx,Ru);

%Creating the Khepera IV object
khep=Khepera4(port,[x0;y0;theta0],param,max_wheels_speed);

%Connecting the robot through bluetooth
[khep,result]=khep.connect();

pose_seq=[];
wr_wl_seq=[];

K_f=length(xr);
% pause(5)
i_time=1;
t=zeros(1,K_f);
time=0;
avg_time=0;


while i_time<=K_f
    tic
    
    khep.x_r=xr(i_time); khep.y_r=yr(i_time);
    khep=khep.set_theoretic_trajectory_tracking();
    
    avg_time=avg_time+toc;
    
    pose_seq=[pose_seq khep.pose()];
    wr_wl_seq=[wr_wl_seq khep.wheels_vel];
    
    while toc<Ts
    end
    %     toc
    time=time+toc;
    t(i_time)=time;
    i_time=i_time+1;
    
end

khep.set_motors_speed(0,0);
khep=khep.disconnect;

err_seq=pose_seq(1:2,:)-[xr;yr];

norm_err=zeros(1,size(err_seq,2));
norm_eng=zeros(1,size(err_seq,2));

for j=1:length(norm_err)
    e_j=err_seq(:,j);
    norm_err(j)=sqrt(e_j'*e_j);
    
    en_j=wr_wl_seq(:,j);
    norm_eng(j)=en_j'*en_j;
    
end


MSE=sum(norm_err.^2)/K_f

F_IAE = griddedInterpolant(t,abs(norm_err));
fun_IAE = @(t) F_IAE(t);
IAE = integral(fun_IAE, t(1), t(end))


F_ISE = griddedInterpolant(t,norm_err.^2);
fun_ISE = @(t) F_ISE(t);
ISE = integral(fun_ISE, t(1), t(end))

F_ITAE = griddedInterpolant(t,t.*abs(norm_err));
fun_ITAE = @(t) F_ITAE(t);
ITAE = integral(fun_ITAE, t(1), t(end))

F_ITSE = griddedInterpolant(t,t.*norm_err.^2);
fun_ITSE = @(t) F_ITSE(t);
ITSE = integral(fun_ITSE, t(1), t(end))

avg_enrg=sum(norm_eng)/(K_f)

compute_time=1;
convergence_time=inf;

for i=1:length(pose_seq)
    qr_k=[xr(i);yr(i)];
    q_k=pose_seq(1:2,i);
    
    if q_k'*q_k>0.95*(qr_k'*qr_k)
        
        convergence_time=inf;
        compute_time=1;
    elseif q_k'*q_k<=0.95*(qr_k'*qr_k)
        convergence_time=(i-1)*Ts;
        compute_time=0;
        
    end
    convergence_time;
    
end


disp('our-8')
save('sim_our_O','pose_seq','wr_wl_seq','t','xr','yr')
