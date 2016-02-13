classdef Battery < robotics.ros.Message
    %Battery MATLAB implementation of irobotcreate2/Battery
    %   This class was automatically generated by
    %   robotics.ros.msg.internal.gen.MessageClassGenerator.
    
    %   Copyright 2016 The MathWorks, Inc.
    
    %#ok<*INUSD>
    
    properties (Constant)
        MessageType = 'irobotcreate2/Battery' % The ROS message type
    end
    
    properties (Constant, Hidden)
        MD5Checksum = 'fea569ed505a7f18307badadb1c26b0c' % The MD5 Checksum of the message definition
    end
    
    properties (Access = protected)
        JavaMessage % The Java message object
    end
    
    properties (Constant, Access = protected)
        StdMsgsHeaderClass = robotics.ros.msg.internal.MessageFactory.getClassForType('std_msgs/Header') % Dispatch to MATLAB class for message type std_msgs/Header
    end
    
    properties (Dependent)
        Header
        PowerCord
        Dock
        Level
        TimeRemaining
    end
    
    properties (Access = protected)
        Cache = struct('Header', []) % The cache for fast data access
    end
    
    properties (Constant, Hidden)
        PropertyList = {'Dock', 'Header', 'Level', 'PowerCord', 'TimeRemaining'} % List of non-constant message properties
        ROSPropertyList = {'dock', 'header', 'level', 'power_cord', 'time_remaining'} % List of non-constant ROS message properties
    end
    
    methods
        function obj = Battery(msg)
            %Battery Construct the message object Battery
            import com.mathworks.toolbox.robotics.ros.message.MessageInfo;
            
            % Support default constructor
            if nargin == 0
                obj.JavaMessage = obj.createNewJavaMessage;
                return;
            end
            
            % Construct appropriate empty array
            if isempty(msg)
                obj = obj.empty(0,1);
                return;
            end
            
            % Make scalar construction fast
            if isscalar(msg)
                % Check for correct input class
                if ~MessageInfo.compareTypes(msg(1), obj.MessageType)
                    error(message('robotics:ros:message:NoTypeMatch', obj.MessageType, ...
                        char(MessageInfo.getType(msg(1))) ));
                end
                obj.JavaMessage = msg(1);
                return;
            end
            
            % Check that this is a vector of scalar messages. Since this
            % is an object array, use arrayfun to verify.
            if ~all(arrayfun(@isscalar, msg))
                error(message('robotics:ros:message:MessageArraySizeError'));
            end
            
            % Check that all messages in the array have the correct type
            if ~all(arrayfun(@(x) MessageInfo.compareTypes(x, obj.MessageType), msg))
                error(message('robotics:ros:message:NoTypeMatchArray', obj.MessageType));
            end
            
            % Construct array of objects if necessary
            objType = class(obj);
            for i = 1:length(msg)
                obj(i,1) = feval(objType, msg(i)); %#ok<AGROW>
            end
        end
        
        function header = get.Header(obj)
            %get.Header Get the value for property Header
            if isempty(obj.Cache.Header)
                obj.Cache.Header = feval(obj.StdMsgsHeaderClass, obj.JavaMessage.getHeader);
            end
            header = obj.Cache.Header;
        end
        
        function set.Header(obj, header)
            %set.Header Set the value for property Header
            validateattributes(header, {obj.StdMsgsHeaderClass}, {'nonempty', 'scalar'}, 'Battery', 'header');
            
            obj.JavaMessage.setHeader(header.getJavaObject);
            
            % Update cache if necessary
            if ~isempty(obj.Cache.Header)
                obj.Cache.Header.setJavaObject(header.getJavaObject);
            end
        end
        
        function powercord = get.PowerCord(obj)
            %get.PowerCord Get the value for property PowerCord
            powercord = logical(obj.JavaMessage.getPowerCord);
        end
        
        function set.PowerCord(obj, powercord)
            %set.PowerCord Set the value for property PowerCord
            validateattributes(powercord, {'logical', 'numeric'}, {'nonempty', 'scalar'}, 'Battery', 'powercord');
            
            obj.JavaMessage.setPowerCord(powercord);
        end
        
        function dock = get.Dock(obj)
            %get.Dock Get the value for property Dock
            dock = logical(obj.JavaMessage.getDock);
        end
        
        function set.Dock(obj, dock)
            %set.Dock Set the value for property Dock
            validateattributes(dock, {'logical', 'numeric'}, {'nonempty', 'scalar'}, 'Battery', 'dock');
            
            obj.JavaMessage.setDock(dock);
        end
        
        function level = get.Level(obj)
            %get.Level Get the value for property Level
            level = single(obj.JavaMessage.getLevel);
        end
        
        function set.Level(obj, level)
            %set.Level Set the value for property Level
            validateattributes(level, {'numeric'}, {'nonempty', 'scalar'}, 'Battery', 'level');
            
            obj.JavaMessage.setLevel(level);
        end
        
        function timeremaining = get.TimeRemaining(obj)
            %get.TimeRemaining Get the value for property TimeRemaining
            timeremaining = typecast(int32(obj.JavaMessage.getTimeRemaining), 'uint32');
        end
        
        function set.TimeRemaining(obj, timeremaining)
            %set.TimeRemaining Set the value for property TimeRemaining
            validateattributes(timeremaining, {'numeric'}, {'nonempty', 'scalar'}, 'Battery', 'timeremaining');
            
            obj.JavaMessage.setTimeRemaining(timeremaining);
        end
    end
    
    methods (Access = protected)
        function resetCache(obj)
            %resetCache Resets any cached properties
            obj.Cache.Header = [];
        end
        
        function cpObj = copyElement(obj)
            %copyElement Implements deep copy behavior for message
            
            % Call default copy method for shallow copy
            cpObj = copyElement@robotics.ros.Message(obj);
            
            % Clear any existing cached properties
            cpObj.resetCache;
            
            % Create a new Java message object
            cpObj.JavaMessage = obj.createNewJavaMessage;
            
            % Iterate over all primitive properties
            cpObj.PowerCord = obj.PowerCord;
            cpObj.Dock = obj.Dock;
            cpObj.Level = obj.Level;
            cpObj.TimeRemaining = obj.TimeRemaining;
            
            % Recursively copy compound properties
            cpObj.Header = copy(obj.Header);
        end
        
        function reload(obj, strObj)
            %reload Called by loadobj to assign properties
            obj.PowerCord = strObj.PowerCord;
            obj.Dock = strObj.Dock;
            obj.Level = strObj.Level;
            obj.TimeRemaining = strObj.TimeRemaining;
            obj.Header = feval([obj.StdMsgsHeaderClass '.loadobj'], strObj.Header);
        end
    end
    
    methods (Access = ?robotics.ros.Message)
        function strObj = saveobj(obj)
            %saveobj Implements saving of message to MAT file
            
            % Return an empty element if object array is empty
            if isempty(obj)
                strObj = struct.empty;
                return
            end
            
            strObj.PowerCord = obj.PowerCord;
            strObj.Dock = obj.Dock;
            strObj.Level = obj.Level;
            strObj.TimeRemaining = obj.TimeRemaining;
            strObj.Header = saveobj(obj.Header);
        end
    end
    
    methods (Static, Access = {?matlab.unittest.TestCase, ?robotics.ros.Message})
        function obj = loadobj(strObj)
            %loadobj Implements loading of message from MAT file
            
            % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = robotics.ros.custom.msggen.irobotcreate2.Battery.empty(0,1);
                return
            end
            
            % Create an empty message object
            obj = robotics.ros.custom.msggen.irobotcreate2.Battery;
            obj.reload(strObj);
        end
    end
end
