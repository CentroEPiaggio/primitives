classdef CustomMsgConsts
    %CustomMsgConsts This class stores all message types
    %   The message types are constant properties, which in turn resolve
    %   to the strings of the actual types.
    
    %   Copyright 2016 The MathWorks, Inc.
    
    properties (Constant)
        ff_fb_control_dd_control = 'ff_fb_control/dd_control'
        irobotcreate2_Battery = 'irobotcreate2/Battery'
        irobotcreate2_Brushes = 'irobotcreate2/Brushes'
        irobotcreate2_Bumper = 'irobotcreate2/Bumper'
        irobotcreate2_Buttons = 'irobotcreate2/Buttons'
        irobotcreate2_Diagnostic = 'irobotcreate2/Diagnostic'
        irobotcreate2_DigitLeds = 'irobotcreate2/DigitLeds'
        irobotcreate2_IRCharacter = 'irobotcreate2/IRCharacter'
        irobotcreate2_Leds = 'irobotcreate2/Leds'
        irobotcreate2_Note = 'irobotcreate2/Note'
        irobotcreate2_PlaySong = 'irobotcreate2/PlaySong'
        irobotcreate2_RoombaIR = 'irobotcreate2/RoombaIR'
        irobotcreate2_RoombaSwitch = 'irobotcreate2/RoombaSwitch'
        irobotcreate2_ScheduleLeds = 'irobotcreate2/ScheduleLeds'
        irobotcreate2_Song = 'irobotcreate2/Song'
        irobotcreate2_WheelDrop = 'irobotcreate2/WheelDrop'
        map_voronoi_Agent_msg = 'map_voronoi/Agent_msg'
    end
    
    methods (Static, Hidden)
        function messageList = getMessageList
            %getMessageList Generate a cell array with all message types.
            %   The list will be sorted alphabetically.
            
            persistent msgList
            if isempty(msgList)
                msgList = cell(17, 1);
                msgList{1} = 'ff_fb_control/dd_control';
                msgList{2} = 'irobotcreate2/Battery';
                msgList{3} = 'irobotcreate2/Brushes';
                msgList{4} = 'irobotcreate2/Bumper';
                msgList{5} = 'irobotcreate2/Buttons';
                msgList{6} = 'irobotcreate2/Diagnostic';
                msgList{7} = 'irobotcreate2/DigitLeds';
                msgList{8} = 'irobotcreate2/IRCharacter';
                msgList{9} = 'irobotcreate2/Leds';
                msgList{10} = 'irobotcreate2/Note';
                msgList{11} = 'irobotcreate2/PlaySong';
                msgList{12} = 'irobotcreate2/RoombaIR';
                msgList{13} = 'irobotcreate2/RoombaSwitch';
                msgList{14} = 'irobotcreate2/ScheduleLeds';
                msgList{15} = 'irobotcreate2/Song';
                msgList{16} = 'irobotcreate2/WheelDrop';
                msgList{17} = 'map_voronoi/Agent_msg';
            end
            
            messageList = msgList;
        end
        
        function serviceList = getServiceList
            %getServiceList Generate a cell array with all service types.
            %   The list will be sorted alphabetically.
            
            persistent svcList
            if isempty(svcList)
                svcList = cell(0, 1);
            end
            
            % The message list was already sorted, so don't need to sort
            % again.
            serviceList = svcList;
        end
    end
end
