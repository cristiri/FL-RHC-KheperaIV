classdef (Abstract) control_parameters
    
    %This class build a data structure containing all the parameters
    %necessary to the control
    
    properties (Abstract,SetAccess = protected)
        Ts
    end
end

