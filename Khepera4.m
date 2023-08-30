classdef Khepera4
    %This class model a Khepera4 object to control Khepera4 robots7
    %Properties:
    % 1) serial_com is a struct containing information to establish a
    %serial communication with the robot.
    % 2) pose is an array containing postions x,y and orientation of the robot [x;y;theta]
    % 3) wheels_vel is an array containing the speed values for right and
    % left wheel of the robot
    properties
        x_r, y_r,theta_r, xp_r, yp_r, xpp_r, ypp_r
    end
    properties (Dependent)
        M, M_inv,v_r,w_r,desired_max_speed_rad
    end
    properties ( SetAccess = protected)
        serial_com, pose, wheels_vel, desired_max_speed, connected, enc_pos_left, enc_pos_right, control_parameters
    end
    properties (Constant)
        wheel_distance = 0.10470;
        wheel_conversion_left = 0.13194 / 19456.0;
        wheel_conversion_right = 0.13194 / 19456.0;
        r=0.0210;
        d=0.10470;
        max_speed_meters=0.813 % m/s
        max_speed_rad=0.813/0.0210;
    end
    
    methods
        function obj = Khepera4(port,pose,control_parameters,desired_max_speed)
            obj.connected=false;
            obj.wheels_vel=[0;0];
            obj.enc_pos_left=0;
            obj.enc_pos_right=0;
            if nargin==0 %Default constructor
                obj.serial_com=serial('/dev/rfcomm0','BaudRate', 115200,'TimeOut',0.3,'StopBits',1);
                obj.pose=[0;0;0];
                obj.control_parameters=STTT_control_parameters();
                obj.x_r=0;
                obj.y_r=0;
                obj.desired_max_speed=600;
            else
                obj.serial_com=serial(port,'BaudRate', 115200,'TimeOut',0.3,'StopBits',1);
                obj.pose=pose;
                obj.control_parameters=control_parameters;
                obj.desired_max_speed=desired_max_speed;
            end
        end
        
        function [obj,result]=connect(obj)
            if obj.connected~=true
                try
                    % open port
                    fopen(obj.serial_com);
                    disp 'Khepera4 successfully connected'
                    result= 1;
                    obj.connected=true;
                    obj.initialize();
                catch
                    % could not open the port
                    disp 'Error, Could not open serial port'
                    result = 0;
                end
            else
                disp 'Khepera4 already connected'
                result=0;
            end
        end
        
        function [obj,result]=disconnect(obj)
            
            if obj.connected==true
                try
                    fclose(obj.serial_com);
                    disp 'Khepera4 successfully disconnected'
                    result = 1;
                    obj.connected=false;
                catch
                    disp 'Error while closing serial port'
                    result = 0;
                end
            else
                disp 'Khepera4 already disconnected'
            end
            
        end
        
        
        function set_motors_speed(obj,w_l,w_r)
            %Sendind the velocity command
            %             while strcmp(obj.serial_com.TransferStatus, 'idle')~=1
            %             end
            fprintf(obj.serial_com,['D,' num2str(w_l) ',' num2str(w_r)],'async');
            fscanf(obj.serial_com);
        end
        
        function [enc_left,enc_right]=read_encoders(obj)
            %Sending the encoder reading command
            fprintf(obj.serial_com,'R','async');
            readasync(obj.serial_com);
            resp=fscanf(obj.serial_com);
            tkns=split(resp,',');
            enc_left=str2double(tkns(2));
            enc_right=str2double(tkns(3));
        end
        
        function reset_encoders(obj)
            fprintf(obj.serial_com,'I','async');
            fscanf(obj.serial_com);
        end
        
        function wheels_speed=read_wheels_speed(obj)
            %Sending the encoder reading command
            fprintf(obj.serial_com,'E','async');
            readasync(obj.serial_com);
            resp=fscanf(obj.serial_com);
            tkns=split(resp,',');
            left_motor=str2double(tkns(2));
            right_motor=str2double(tkns(3));
            wheels_speed=[right_motor;left_motor];
        end
        
        function ultrasound_off(obj)
            fprintf(obj.serial_com,'U,0','async');
            fscanf(obj.serial_com);
        end
        
        function status=get_battery_status(obj)
            fprintf(obj.serial_com,'V,5','async');
            status=fscanf(obj.serial_com);
        end
        
        function initialize(obj)
            %1st random command (the first sent command seems to be ignored)
            fprintf(obj.serial_com,'V,5','async');
            fscanf(obj.serial_com);
            
            %deactivating ultrasound
            fprintf(obj.serial_com,'U,0','async');
            fscanf(obj.serial_com);
            
            %resetting encoders
            fprintf(obj.serial_com,'I','async');
            fscanf(obj.serial_com);
        end
        
        
        function obj=update_odometry(obj)
            %update encoder positions
            [enc_pos_left_new, enc_pos_right_new]=obj.read_encoders();
            delta_pos_left = enc_pos_left_new - obj.enc_pos_left;
            delta_pos_right = enc_pos_right_new - obj.enc_pos_right;
            delta_left = delta_pos_left * obj.wheel_conversion_left;
            delta_right = delta_pos_right * obj.wheel_conversion_right;
            delta_theta = (delta_right - delta_left) / obj.wheel_distance;
            theta2 = obj.pose(3) + delta_theta * 0.5;
            delta_x = (delta_left + delta_right) * 0.5 * cos(theta2);
            delta_y = (delta_left + delta_right) * 0.5 * sin(theta2);
            obj.pose(1)=obj.pose(1)+delta_x;
            obj.pose(2)=obj.pose(2)+delta_y;
            obj.pose(3)=obj.pose(3)+delta_theta;
            obj.enc_pos_left=enc_pos_left_new;
            obj.enc_pos_right=enc_pos_right_new;
        end
        
        function desired_max_speed_rad=get.desired_max_speed_rad(obj)
            desired_max_speed_rad=obj.desired_max_speed*obj.max_speed_rad/1200;
            %               desired_max_speed_rad=obj.desired_max_speed;
        end
        
        function v_r=get.v_r(obj)
            v_r=sqrt(obj.xp_r.^2+obj.yp_r.^2);
        end
        
        function w_r=get.w_r(obj)
            w_r=(obj.ypp_r.*obj.xp_r-obj.xpp_r.*obj.yp_r)./(obj.xp_r.^2+obj.yp_r.^2);
        end
        
        function M=get.M(obj)
            M=[[obj.r/2 obj.r/2];[obj.r/obj.d -obj.r/obj.d]];
        end
        
        function M_inv=get.M_inv(obj)
            M_inv=inv(obj.M);
        end
        
        
        function obj=set_theoretic_trajectory_tracking(obj)
            
            function [H_poly,g_poly] = input_polyhedron(M_theta,M,max_wheels_speed)
                
                g_poly=max_wheels_speed*ones(4,1);
                M_box=[-1 0; 0 -1; 1 0; 0 1];
                
                M_rhombus=M_box/M;
                
                H_poly=M_rhombus*M_theta;
            end
            
            function [Q_sol,F_sol,comp_time] = kothare_traj_track( Ad,Bd,x0,Q1,R,H)
                n=size(Ad,1);
                m=size(Bd,2);
                
                setlmis([])
                Q = lmivar(1,[n 1]);
                Y = lmivar(2,[m n]);
                gamma=lmivar(1,[1 1]);
                
                %LMI constraints
                lmiterm([-1 1 1 Q],1,1);%constraint 1
                
                lmiterm([-2,1,1,Q],1,1);  %constraint 2 block (1,1)
                lmiterm([-2,2,1,Q],Ad,1); %constraint 2 block (2,1)
                lmiterm([-2,2,1,Y],Bd,1); %constraint 2 block (2,1)
                lmiterm([-2,2,2,Q],1,1);  %constraint 2 block (2,2)
                lmiterm([-2,3,1,Q],sqrtm(Q1),1); %constraint 2 block (3,1)
                lmiterm([-2,3,2,0],zeros(n,n));  %constraint 2 block (3,2)
                lmiterm([-2,3,3,gamma],1,eye(n));%constraint 2 block (3,3)
                lmiterm([-2,4,1,Y],sqrtm(R),1);  %constraint 2 block (4,1)
                lmiterm([-2,4,2,0],zeros(m,n));  %constraint 2 block (4,2)
                lmiterm([-2,4,3,0],zeros(m,n));  %constraint 2 block (4,3)
                lmiterm([-2,4,4,gamma],1,eye(m));%constraint 2 block (4,4)
                
                lmiterm([-3,1,1,0],1); %constraint 3 block (1,1)
                lmiterm([-3,2,1,0],x0); %constraint 3 block (2,1)
                lmiterm([-3,2,2,Q],1,1);%constraint 3 block (2,2)
                
                lmiterm([-4,1,1,Q],1,1); %constraint 4 block (1,1)
                lmiterm([-4,2,1,Y],H(1,:),1); %constraint 4 block (2,1)
                lmiterm([-4,2,2,0],1); %constraint 4 block (2,2)
                
                lmiterm([-5,1,1,Q],1,1); %constraint 5 block (1,1)
                lmiterm([-5,2,1,Y],H(2,:),1); %constraint 5 block (2,1)
                lmiterm([-5,2,2,0],1); %constraint 5 block (2,2)
                
                lmiterm([-6,1,1,Q],1,1); %constraint 6 block (1,1)
                lmiterm([-6,2,1,Y],H(3,:),1); %constraint 6 block (2,1)
                lmiterm([-6,2,2,0],1); %constraint 6 block (2,2)
                
                lmiterm([-7,1,1,Q],1,1); %constraint 7 block (1,1)
                lmiterm([-7,2,1,Y],H(4,:),1); %constraint 7 block (2,1)
                lmiterm([-7,2,2,0],1); %constraint 7 block (2,2)
                
                lmis = getlmis;
                
                % c = defcx(lmis,8,Q,Y,gamma)
                
                c=zeros(8,1);
                c(8)=1;
                options = [1e-10,0,0,0,1];
                [comp_time,sol]=mincx(lmis,c,options);
                
                Q_sol = dec2mat(lmis,sol,Q);
                Y_sol = dec2mat(lmis,sol,Y);
                
                F_sol=Y_sol/Q_sol;
            end
            
            obj=obj.update_odometry();
            
            x=obj.pose(1);
            y=obj.pose(2);
            theta=obj.pose(3);
            
            x_FL=[obj.x_r-x;obj.y_r-y];
            
            %             theta_new=theta_transform(theta);
            
            
            M_theta = [cos(theta) sin(theta); -sin(theta)/obj.control_parameters.b cos(theta)/obj.control_parameters.b];
            
            
            [H,g]=input_polyhedron(M_theta,obj.M,obj.desired_max_speed_rad);
            
            
            [~,F_kot,~]=kothare_traj_track(obj.control_parameters.Ad,obj.control_parameters.Bd,x_FL,obj.control_parameters.Qx,obj.control_parameters.Ru,H/g(1));
            
            u_k=-F_kot*x_FL;
            
            vw=M_theta*u_k;
            
            wrwl=obj.M\vw;
            
            wrwl=wrwl*1200/obj.max_speed_rad;
            
            obj.set_motors_speed(wrwl(2),wrwl(1));
            obj.wheels_vel=[wrwl(1);wrwl(2)];
        end
        
        function obj=Kuhne_trajectory_tracking(obj)
            
            function Ak = A_k(vrk,thetark,Tc)
                
                %Compute A(k)
                Ak=[1 0 -vrk*sin(thetark)*Tc;
                    0 1 vrk*cos(thetark)*Tc;
                    0 0 1];
                
            end
            
            function Bk = B_k(thetark,Tc)
                
                %Compute B(k)
                Bk=[cos(thetark)*Tc 0;
                    sin(thetark)*Tc 0;
                    0 Tc];
                
            end
            
            
            function [Ap, Bp] = matrix_prediction(thetar,vr,Tc)
                %Compute the Ak Bk prediction over horizon N
                Ap=[];
                Bp=[];
                for i=1:length(vr)
                    
                    Ak=A_k(vr(i),thetar(i),Tc);
                    
                    Ap=[Ap;Ak];
                    
                end
                for i=1:length(vr)
                    
                    Bk=B_k(thetar(i),Tc);
                    
                    Bp=[Bp;Bk];
                    
                end
            end
            
            function [H,g]=input_polyhedron(M_inv,max_wheels_speed)
                H_square=[-1 0;0 -1;1 0;0 1];
                H=H_square*M_inv;
                H=H/max_wheels_speed;
                g=ones(4,1);
            end
            
            
            function [ubark] = Kuhne_MPC(Ap,Bp,Q,R,xtilde,N,ubar_rk,H,g)
                %Model Predictive Control optimization with Polyhedral input constraints
                
                
                function [Abark] = A_bark(Ap)
                    n=size(Ap,2);
                    Np=length(Ap)/n;
                    Abark=[];
                    
                    for i=1:Np
                        
                        Apj=eye(3,3);
                        for j=1:i
                            Apj=Apj*Ap(3*(j-1)+1:3*j,1:n);
                        end
                        Abark=[Abark;Apj];
                    end
                end
                
                function [Bbark] = B_bark(Ap,Bp)
                    
                    %Compute the matrix Babr(k)
                    
                    n=size(Ap,2);
                    m=size(Bp,2);
                    Np=length(Ap)/n;
                    Bbark=zeros(Np*n,Np*m);
                    
                    for k=1:Np
                        %Main diagonal elements of the matric Bbar(k)
                        Bbark(n*(k-1)+1:n*k,2*k-1:2*k)=Bp(n*(k-1)+1:n*k,1:m);
                        
                    end
                    %Algorithm that fills the lower triangle of the matrix Bbar(k)
                    
                    L=n;
                    M=0;
                    
                    %Vector for indexing Bp(k)
                    s=1;
                    ind2=zeros(1,length(Bp));
                    
                    for z=1:2:length(ind2)
                        ind2(z)=s;
                        s=s+n;
                    end
                    
                    for i=4:n:n*Np
                        for j=1:2:i-L
                            
                            Abloc=eye(n);
                            
                            
                            
                            %k indexing in Ap(k) matrix
                            s=4;
                            ind=zeros(1,length(Ap));
                            
                            for z=1:2:length(ind)
                                ind(z)=s;
                                s=s+n;
                            end
                            
                            
                            
                            for k=ind(j):n:i
                                Abloc=Abloc*Ap(k:k+2,1:n);
                            end
                            
                            ABbloc=Abloc*Bp(ind2(j):ind2(j)+2,1:m);
                            
                            
                            Bbark(i:i+2,j:j+1)=ABbloc;
                            
                        end
                        
                        L=L+1;
                        M=M+1;
                    end
                    
                end
                
                function Qbar = Q_bar(Q,N)
                    
                    %Build the Qbar matrix
                    %N is the rpediciton horizon
                    n=size(Q,1);
                    Qbar=zeros(N*n,N*n);
                    
                    for i=1:n:length(Qbar)
                        Qbar(i:i+n-1,i:i+n-1)=Q;
                    end
                    
                end
                
                function Rbar = R_bar(R,N)
                    
                    %Build the Rbar matrix
                    %N is the rpediciton horizon
                    m=size(R,1);
                    Rbar=zeros(N*m,N*m);
                    
                    for i=1:m:length(Rbar)
                        Rbar(i:i+m-1,i:i+m-1)=R;
                    end
                    
                end
                
                
                Abark=A_bark(Ap);
                Bbark=B_bark(Ap,Bp);
                Qbar=Q_bar(Q,N);
                Rbar=R_bar(R,N);
                
                
                Hk=2*(Bbark'*Qbar*Bbark+Rbar);
                fk=2*Bbark'*Qbar*Abark*xtilde;
                % dk=xtilde'*Abark'*Qbar*Abark*xtilde;
                
                
                
                %Constraints variables definition
                
                Hbar=[];
                for i=1:N
                    Hbar=blkdiag(Hbar,H);
                end
                
                gbar=[];
                for i=1:N
                    gbar=[gbar;g];
                end
                
                
                %MPC solution
                Aeq = [];
                beq = [];
                lb = [];
                ub = [];
                x0 = [];
                options = optimoptions('quadprog','Display','off');
                
                % tic
                [ubark,~,~,~,~] = quadprog(Hk,fk,Hbar,gbar,Aeq,beq,lb,ub,x0,options);
                
            end
            
            obj=obj.update_odometry();
            N=obj.control_parameters.N;
            
            x=obj.pose(1);
            y=obj.pose(2);
            theta=obj.pose(3);
            
            %Current portion of the trajectory x(k|k), x(k+1|k) .... x(k+N-1|k)
            vrk=obj.v_r(1:N);
            wrk=obj.w_r(1:N);
            thetark=obj.theta_r(1:N);
            
            %Tracking error
            x_tilde=[x;y;theta]-[obj.x_r(1);obj.y_r(1);obj.theta_r(1)];
            
            
            obj.x_r=obj.x_r(2:end);
            obj.y_r=obj.y_r(2:end);
            obj.theta_r=obj.theta_r(2:end);
            obj.xp_r=obj.xp_r(2:end);
            obj.yp_r=obj.yp_r(2:end);
            obj.xpp_r=obj.xpp_r(2:end);
            obj.ypp_r=obj.ypp_r(2:end);
            
            %State Prediction
            [Ap,Bp]=matrix_prediction(thetark,vrk,obj.control_parameters.Ts);
            
            
            %Input Constraints
            %%u_bar_r(k)
            ubar_rk=[];
            for j=1:length(vrk)
                ubar_rk=[ubar_rk ;[vrk(j);wrk(j)]];
            end
            
            [H,g]=input_polyhedron(obj.M_inv,obj.desired_max_speed_rad);
            
            Q=obj.control_parameters.Qx;
            R=obj.control_parameters.Ru;
            
            %MPC control law
            [usol]=Kuhne_MPC(Ap,Bp,Q,R,x_tilde,N,ubar_rk,H,g);
            
            %Apply the first element of the sequence (RHC logic)
            uk=usol(1:2);
            vw=uk;
            
            wrwl=obj.M_inv*vw;
            wrwl=wrwl*1200/obj.max_speed_rad;
            
            
            obj.set_motors_speed(wrwl(2),wrwl(1));
            obj.wheels_vel=[wrwl(1);wrwl(2)];
            
        end
        
        
        function obj=DeLuca_trajectory_tracking(obj)
            
            
            obj=obj.update_odometry();
            
            x=obj.pose(1);
            y=obj.pose(2);
            theta=obj.pose(3);
            
            
            xp=obj.control_parameters.csi*cos(theta);
            yp=obj.control_parameters.csi*sin(theta);
            
            
            
            kp1=obj.control_parameters.kp1;
            kp2=obj.control_parameters.kp2;
            kd1=obj.control_parameters.kd1;
            kd2=obj.control_parameters.kd2;
            
            
            mode=obj.control_parameters.mode;
            if mode=="TT"
                u1=obj.xpp_r(1)+kp1*(obj.x_r(1)-x)+kd1*(obj.xp_r(1)-xp);
                u2=obj.ypp_r(1)+kp2*(obj.y_r(1)-y)+kd2*(obj.yp_r(1)-yp);
            elseif mode== "PF"
                u1=kp1*(obj.x_r(1)-x)-kd1*xp;
                u2=kp2*(obj.y_r(1)-y)-kd2*yp;
            end
            
            
            
            obj.x_r=obj.x_r(2:end);
            obj.y_r=obj.y_r(2:end);
            obj.xp_r=obj.xp_r(2:end);
            obj.yp_r=obj.yp_r(2:end);
            obj.xpp_r=obj.xpp_r(2:end);
            obj.ypp_r=obj.ypp_r(2:end);
            
            
            
            csip=u1*cos(theta)+u2*sin(theta);
            obj.control_parameters.csi=obj.control_parameters.csi+csip*obj.control_parameters.Ts;
            
            v=obj.control_parameters.csi;
            w=(u2*cos(theta)-u1*sin(theta))/obj.control_parameters.csi;
            
            wrmax=obj.desired_max_speed_rad;
            wlmax=wrmax;
            
            %Saturation
            if w>2*wrmax*obj.r/obj.d
                w=2*wrmax*obj.r/obj.d;
            end
            if w<-(2*wrmax*obj.r/obj.d)
                w=-2*wrmax*obj.r/obj.d;
            end
            
            if v>obj.r*wrmax-obj.d/2*w
                v=obj.r*wrmax-obj.d/2*w;
            end
            if v>obj.r*wlmax+obj.d/2*w
                v=obj.r*wlmax+obj.d/2*w;
            end
            
            if v<-(obj.r*wrmax-obj.d/2*w)
                v=-(obj.r*wrmax-obj.d/2*w);
            end
            if v<-(obj.r*wlmax+obj.d/2*w)
                v=-(obj.r*wlmax+obj.d/2*w);
            end
            
            
            
            wrwl=obj.M_inv*[v;w];
            wrwl=wrwl*1200/obj.max_speed_rad;
            
            
            obj.set_motors_speed(wrwl(2),wrwl(1));
            obj.wheels_vel=[wrwl(1);wrwl(2)];
            
        end
        
        
        
        function obj=Blazic_trajectory_tracking(obj)
            
            obj=obj.update_odometry();
            
            x=obj.pose(1);
            y=obj.pose(2);
            theta=obj.pose(3);
            
            
            kx=obj.control_parameters.kx;
            kc=obj.control_parameters.kc;
            ks=obj.control_parameters.ks;
            ac=obj.control_parameters.ac;
            nc=obj.control_parameters.nc;
            
            
            ex=cos(theta)*(obj.x_r(1)-x)+sin(theta)*(obj.y_r(1)-y);
            ey=-sin(theta)*(obj.x_r(1)-x)+cos(theta)*(obj.y_r(1)-y);
            es=sin(obj.theta_r(1)-theta);
            ec=cos(obj.theta_r(1)-theta)-1;
            
            vb=kx*ex;
            wb=kc*obj.v_r(1)*ey*(1+ec/ac)^2+ks*es*((1+ec/ac)^2)^nc;
            
            
            v=obj.v_r(1)*cos(obj.theta_r(1)-theta)+vb;
            w=obj.w_r(1)+wb;
            
            obj.x_r=obj.x_r(2:end);
            obj.y_r=obj.y_r(2:end);
            obj.xp_r=obj.xp_r(2:end);
            obj.yp_r=obj.yp_r(2:end);
            obj.xpp_r=obj.xpp_r(2:end);
            obj.ypp_r=obj.ypp_r(2:end);
            obj.theta_r=obj.theta_r(2:end);
            
            wrmax=obj.desired_max_speed_rad;
            wlmax=wrmax;
            
            
            if w>2*wrmax*obj.r/obj.d
                w=2*wrmax*obj.r/obj.d;
            end
            if w<-(2*wrmax*obj.r/obj.d)
                w=-2*wrmax*obj.r/obj.d;
            end
            
            if v>obj.r*wrmax-obj.d/2*w
                v=obj.r*wrmax-obj.d/2*w;
            end
            if v>obj.r*wlmax+obj.d/2*w
                v=obj.r*wlmax+obj.d/2*w;
            end
            
            if v<-(obj.r*wrmax-obj.d/2*w)
                v=-(obj.r*wrmax-obj.d/2*w);
            end
            if v<-(obj.r*wlmax+obj.d/2*w)
                v=-(obj.r*wlmax+obj.d/2*w);
            end
            
            wrwl=obj.M_inv*[v;w];
            wrwl=wrwl*1200/obj.max_speed_rad;
            
            
            obj.set_motors_speed(wrwl(2),wrwl(1));
            obj.wheels_vel=[wrwl(1);wrwl(2)];
            
            
        end
        
        function obj=old_trajectory_tracking(obj)
            
            function [theta_fit,pI,k] = converti_theta(theta)
                pI=abs(fix(theta/pi));
                if mod(pI,2)==0
                    if pI~=0
                        pI=pI-1;
                    end
                elseif pI==-theta/pi
                    if pI==1
                        pI=0;
                    else
                        pI=pI-2;
                    end
                end
                k=sign(theta)*(fix(pI/2)+mod(pI,2));
                theta_fit=theta-2*k*pi;
                
                
            end
            
            function [indice] = indice_mat_vinc2(theta)
                %funzione che irceve in ingresso un valore di theta espresso in radianti
                %(-2*pi<theta<2*pi) e restituisce la posizione di theta nella matrice dei
                %vincoli
                
                if theta==0
                    indice=100;
                else
                    Tc=pi/100;
                    a=floor(theta/Tc);
                    indice=101+a;
                    
                    if indice==0
                        indice=200;
                    elseif indice==201
                        indice=1;
                    end
                end
            end
            
            function [Q_sol,F_sol] = kothare3p( Ad,Bd,x0,u_vin,Q1,R)
                n=size(Ad,1);
                m=size(Bd,2);
                
                setlmis([])
                Q = lmivar(1,[n 1]);
                Y = lmivar(2,[m n]);
                [X1,~,sX1]=lmivar(1,[1 1]);
                [X2,~,sX2]=lmivar(1,[1 1]);
                [X,~,~] = lmivar(3,[sX1,0;0,sX2]);
                gamma=lmivar(1,[1 1]);
                
                %LMI constraints
                lmiterm([-1 1 1 Q],1,1);%constraint 1
                
                lmiterm([-2,1,1,Q],1,1);  %constraint 2 block (1,1)
                lmiterm([-2,2,1,Q],Ad,1); %constraint 2 block (2,1)
                lmiterm([-2,2,1,Y],Bd,1); %constraint 2 block (2,1)
                lmiterm([-2,2,2,Q],1,1);  %constraint 2 block (2,2)
                lmiterm([-2,3,1,Q],sqrtm(Q1),1); %constraint 2 block (3,1)
                lmiterm([-2,3,2,0],zeros(n,n));  %constraint 2 block (3,2)
                lmiterm([-2,3,3,gamma],1,eye(n));%constraint 2 block (3,3)
                lmiterm([-2,4,1,Y],sqrtm(R),1);  %constraint 2 block (4,1)
                lmiterm([-2,4,2,0],zeros(m,n));  %constraint 2 block (4,2)
                lmiterm([-2,4,3,0],zeros(m,n));  %constraint 2 block (4,3)
                lmiterm([-2,4,4,gamma],1,eye(m));%constraint 2 block (4,4)
                
                lmiterm([-3,1,1,0],1); %constraint 3 block (1,1)
                lmiterm([-3,2,1,0],x0); %constraint 3 block (2,1)
                lmiterm([-3,2,2,Q],1,1);%constraint 3 block (2,2)
                
                lmiterm([-4,1,1,X],1,1); %constraint 4 block (1,1)
                lmiterm([-4,2,1,-Y],1,1);%constraint 4 block (2,1)
                lmiterm([-4,2,2,Q],1,1); %constraint 4 block (2,2)
                
                lmiterm([5,1,1,X1],1,1);
                lmiterm([5,1,1,0],-u_vin(1)^2)
                
                lmiterm([6,1,1,X2],1,1);
                lmiterm([6,1,1,0],-u_vin(2)^2);
                
                
                lmis = getlmis;
                
                c=zeros(10,1);
                c(10)=1;
                options = [1e-6,0,0,0,1];
                [~,sol]=mincx(lmis,c,options);
                Q_sol = dec2mat(lmis,sol,Q);
                Y_sol = dec2mat(lmis,sol,Y);
                
                F_sol=Y_sol/Q_sol;
                
                
            end
            
            b=obj.control_parameters.b;
            Ad=obj.control_parameters.Ad;
            Bd=obj.control_parameters.Bd;
            
            obj=obj.update_odometry();
            
            x=obj.pose(1);
            y=obj.pose(2);
            theta=obj.pose(3);
            
            
            x_FL=[obj.x_r-x;obj.y_r-y];
            
            theta_cerca=converti_theta(theta);
            indice_cerca=indice_mat_vinc2(theta_cerca);
            u_vin=obj.control_parameters.vincoli_offline(:,indice_cerca);
            [~,Fkot]=kothare3p(Ad,Bd,x_FL,u_vin,obj.control_parameters.Qx,obj.control_parameters.Ru);
            
            uk=-Fkot*x_FL;
            
            
            
            Mc=[[cos(theta) sin(theta)];[-(sin(theta)/b) (cos(theta)/b)]];
            
            vw=Mc*uk;
            
            
            wrwl=obj.M\vw;
            
            wrwl=wrwl*1200/obj.max_speed_rad;
            
            obj.set_motors_speed(wrwl(2),wrwl(1));
            obj.wheels_vel=[wrwl(1);wrwl(2)];
            
            
        end
        
        
        
        
        
        function obj=Klancar_trajectory_tracking(obj)
            
            function Ak = A_k(vrk,wrk,Tc)
                
                %Compute A(k)
                
                A=[0 wrk 0;
                    -wrk 0 vrk;
                    0 0 0];
                
                Ak=eye(3)+A*Tc;
                
            end
            
            
            function Bk = B_k(Tc)
                
                %Compute B(k)
                B=[-1 0;
                    0 0;
                    0 -1];
                
                Bk=B*Tc;
                
            end
            
            
            function [Ap, Bp] = matrix_prediction(vr,wr,Tc)
                %Compute the Ak Bk prediction over horizon N
                Ap=[];
                Bp=[];
                
                for i=1:length(vr)
                    
                    
                    Ak=A_k(vr(i),wr(i),Tc);
                    
                    Ap=[Ap;Ak];
                    
                end
                
                for i=1:length(vr)
                    
                    Bk=B_k(Tc);
                    
                    Bp=[Bp;Bk];
                    
                end
                
            end
            
            function [Kmpc] = Klancar_MPC(Ap,Ar,Bp,Q,R,N)
                %Model Predictive Control optimization with Polyhedral input constraints
                
                
                function [Abark] = A_bark(Ap)
                    
                    
                    n=size(Ap,2);
                    Np=length(Ap)/n;
                    Abark=[];
                    
                    for i=1:Np
                        
                        Apj=eye(3,3);
                        for j=1:i
                            
                            Apj=Apj*Ap(3*(j-1)+1:3*j,1:n);
                            
                        end
                        Abark=[Abark;Apj];
                    end
                    
                end
                
                function [Bbark] = B_bark(Ap,Bp)
                    
                    %Compute the matrix Babr(k)
                    
                    n=size(Ap,2);
                    m=size(Bp,2);
                    Np=length(Ap)/n;
                    Bbark=zeros(Np*n,Np*m);
                    
                    debug_ij=[];
                    debugN=[];
                    for k=1:Np
                        %Main diagonal elements of the matric Bbar(k)
                        Bbark(n*(k-1)+1:n*k,2*k-1:2*k)=Bp(n*(k-1)+1:n*k,1:m);
                        
                    end
                    
                    
                    
                    %Algorithm that fills the lower triangle of the matrix Bbar(k)
                    
                    L=n;
                    M=0;
                    
                    %Vector for indexing Bp(k)
                    s=1;
                    ind2=zeros(1,length(Bp));
                    
                    for z=1:2:length(ind2)
                        ind2(z)=s;
                        s=s+n;
                    end
                    
                    
                    for i=4:n:n*Np
                        
                        for j=1:2:i-L
                            
                            
                            Abloc=eye(n);
                            
                            
                            
                            %k indexing in Ap(k) matrix
                            s=4;
                            ind=zeros(1,length(Ap));
                            
                            for z=1:2:length(ind)
                                ind(z)=s;
                                s=s+n;
                            end
                            
                            
                            
                            for k=ind(j):n:i
                                Abloc=Abloc*Ap(k:k+2,1:n);
                            end
                            
                            ABbloc=Abloc*Bp(ind2(j):ind2(j)+2,1:m);
                            
                            
                            Bbark(i:i+2,j:j+1)=ABbloc;
                            
                        end
                        
                        L=L+1;
                        M=M+1;
                    end
                    
                end
                
                function Qbar = Q_bar(Q,N)
                    
                    %Build the Qbar matrix
                    %N is the rpediciton horizon
                    n=size(Q,1);
                    Qbar=zeros(N*n,N*n);
                    
                    for i=1:n:length(Qbar)
                        Qbar(i:i+n-1,i:i+n-1)=Q;
                    end
                    
                end
                
                function Rbar = R_bar(R,N)
                    
                    %Build the Rbar matrix
                    %N is the rpediciton horizon
                    m=size(R,1);
                    Rbar=zeros(N*m,N*m);
                    
                    for i=1:m:length(Rbar)
                        Rbar(i:i+m-1,i:i+m-1)=R;
                    end
                    
                end
                
                
                function [Arbark] = Ar_bark(Ar,N)
                    
                    
                    
                    Arbark=[];
                    
                    for i=1:N
                        
                        Apj=eye(3,3);
                        for j=1:i
                            
                            Apj=Apj*Ar;
                            
                        end
                        Arbark=[Arbark;Apj];
                    end
                    
                end
                
                
                
                
                
                %Model Predictive Control optimization with Polyhedral input constraints
                m=size(Bp,2);
                
                Abark=A_bark(Ap);
                Bbark=B_bark(Ap,Bp);
                Qbar=Q_bar(Q,N);
                Rbar=R_bar(R,N);
                Arbar=Ar_bark(Ar,N);
                
                U_B=(Bbark'*Qbar*Bbark+Rbar)\(Bbark'*Qbar*(Arbar-Abark));
                
                Kmpc=U_B(1:m,:);
                
            end
            
            obj=obj.update_odometry();
            N=obj.control_parameters.N;
            
            x=obj.pose(1);
            y=obj.pose(2);
            theta=obj.pose(3);
            
            %Current portion of the trajectory x(k|k), x(k+1|k) .... x(k+N-1|k)
            vrk=obj.v_r(1:N);
            wrk=obj.w_r(1:N);
            
            %Tracking error
            x_tilde=[obj.x_r(1);obj.y_r(1);obj.theta_r(1)]-[x;y;theta];
            
            
            obj.x_r=obj.x_r(2:end);
            obj.y_r=obj.y_r(2:end);
            obj.theta_r=obj.theta_r(2:end);
            obj.xp_r=obj.xp_r(2:end);
            obj.yp_r=obj.yp_r(2:end);
            obj.xpp_r=obj.xpp_r(2:end);
            obj.ypp_r=obj.ypp_r(2:end);
            
            %State Prediction
            [Ap,Bp]=matrix_prediction(vrk,wrk,obj.control_parameters.Ts);
            
            
            
            Q=obj.control_parameters.Qx;
            R=obj.control_parameters.Ru;
            Ar=obj.control_parameters.Ar;
            
            %MPC control law
            [K_mpc]=Klancar_MPC(Ap,Ar,Bp,Q,R,N);
            
            %Apply the first element of the sequence (RHC logic)
            e_k=[cos(theta) sin(theta) 0; -sin(theta) cos(theta) 0; 0 0 1]*x_tilde;
            
            uk=K_mpc*e_k;
            
            vw=uk;
            
            wrwl=obj.M_inv*vw;
            
            
            wrmax=obj.desired_max_speed_rad;
            wlmax=wrmax;
            
            if wrwl(1)>wrmax
                wrwl(1)=wrmax;
            elseif wrwl(1)<-wrmax
                wrwl(1)=-wrmax;
            end
            if wrwl(2)>wlmax
                wrwl(2)=wlmax;
            elseif wrwl(2)<-wlmax
                wrwl(2)=-wlmax;
            end
            
            
            
            wrwl=wrwl*1200/obj.max_speed_rad;
            
            
            obj.set_motors_speed(wrwl(2),wrwl(1));
            obj.wheels_vel=[wrwl(1);wrwl(2)];
            
        end
        
        
        
    end
    
end

