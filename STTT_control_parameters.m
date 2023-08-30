classdef STTT_control_parameters < control_parameters
    
    
    properties (SetAccess=protected)
        Ts,b,Qx,Ru,Ad,Bd
    end
    
    methods
        function obj=STTT_control_parameters(Ts,b,Qx,Ru)
            if nargin==0
                obj.Ts=0.2;
                obj.b=0.1;
                obj.Qx=eye(2);
                obj.Ru=0.01*eye(2);
            else
                
                if Ts<=0
                    error('The sampling time must be a positive real number')
                end
                if b==0
                    error('The traslation factor b must not be zero')
                end
                if size(Qx,1)~=2 || size(Qx,2)~=2
                    error('The weight magtrix Qx must be 2x2')
                end
                if size(Ru,1)~=2 || size(Ru,2)~=2
                    error('The weight magtrix Ru must be 2x2')
                end
                obj.Ts=Ts;
                obj.b=b;
                obj.Qx=Qx;
                obj.Ru=Ru;
            end
            obj.Ad=eye(2);
            obj.Bd=obj.Ts*eye(2);
            
        end
    end
end

