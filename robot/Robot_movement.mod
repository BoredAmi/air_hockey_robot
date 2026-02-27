MODULE Cymbergaj
    !==================================================
    ! AIR HOCKEY ROBOT SYSTEM (UDP)
    !==================================================
    
    ! Socket communication variables
    VAR socketdev udp_socket;
    VAR rawbytes received_data;
    VAR num command_code;
    VAR num x_coord;
    VAR num y_coord;
    
    ! Movement parameters - optimized for speed
    CONST speeddata move_speed := v2000;
    CONST speeddata fast_speed := v7000;
    CONST zonedata move_zone := z1;
    ! variables
    PERS wobjdata current_wobject;
    VAR robtarget target_position;
    CONST robtarget home:=[[0,0,0],[0.00693058,0.998893,-0.0460796,-0.00642376],[-1,0,1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    TASK PERS wobjdata test_reakcji:=[FALSE,TRUE,"",[[265.777,-107.621,116.539],[0.999851,0.0129638,0.00494614,-0.0102413]],[[0,0,0],[1,0,0,0]]];
    VAR bool informed:= false;
    
    !==================================================
    ! MAIN PROCEDURE - UDP server initialization
    !==================================================
    PROC main()
        AccSet 100, 100;
        
        ! Create UDP socket on port 1025
        SocketCreate udp_socket \UDP ;
        SocketBind udp_socket, "0.0.0.0", 1025;
        
        ! Main UDP command loop
        ProcessUDPMessages;
        SocketClose udp_socket;
    ENDPROC
    !==================================================
    ! UDP MESSAGE PROCESSING - Fast command loop
    !==================================================
    PROC ProcessUDPMessages()
        VAR string client_ip;
        VAR num client_port;
        
        WHILE TRUE DO
            SocketReceiveFrom udp_socket \RawData:=received_data, client_ip, client_port \Time:=0.001;
            
            IF RawBytesLen(received_data) > 0 THEN
                ParseAndExecuteCommand;
            ENDIF
        ENDWHILE
        
        ERROR
        IF ERRNO = ERR_SOCK_TIMEOUT THEN
            RETURN;
        ELSE
            TPWrite "UDP error: " + NumToStr(ERRNO, 0);
            RETURN;
        ENDIF
    ENDPROC
    
    !==================================================
    ! COMMAND PARSER - Routes commands to handlers
    !==================================================
    PROC ParseAndExecuteCommand()
        IF RawBytesLen(received_data) >= 4 THEN
            UnpackRawBytes received_data, 1, command_code \IntX:=INT;
            !TPWrite "Received command code: " + NumToStr(command_code, 0);
            TEST command_code
            CASE 0:
                ! START command
                setup;
            CASE 1:
                ! STOP command
                HandleStopCommand;
            CASE 2:
                ! MOVE command
                IF RawBytesLen(received_data) >= 12 THEN
                    UnpackRawBytes received_data, 5, x_coord \Float4;
                    UnpackRawBytes received_data, 9, y_coord \Float4;
                    HandleMoveCommand x_coord, y_coord;
                ELSE
                    TPWrite "MOVE command: insufficient data";
                ENDIF
            DEFAULT:
                TPWrite "Unknown command code: " + NumToStr(command_code, 0);
            ENDTEST
        ELSE
            TPWrite "Data too short: " + NumToStr(RawBytesLen(received_data), 0) + " bytes";
        ENDIF
        
        ERROR
        TPWrite "Command error";
    ENDPROC
    
    
    !==================================================
    ! MOVEMENT HANDLER - Processes MOVE,X,Y commands
    !==================================================
    PROC HandleMoveCommand(num x_coord, num y_coord)
        !TPWrite "Moving to X=" + NumToStr(x_coord, 2) + ", Y=" + NumToStr(y_coord, 2);
        target_position := offs(home, x_coord, y_coord, 0);
        MoveL target_position, fast_speed, move_zone,  tool0,\WObj:=test_reakcji ;
        
    ENDPROC
    
    !==================================================
    ! STOP - Returns robot to safe position
    !==================================================
    PROC HandleStopCommand()
        TPWrite "Stop command received";
        target_position := home;
        MoveL Offs(target_position, 0, 0, 0), fast_speed, move_zone, tool0,\WObj:=test_reakcji ;
    ENDPROC
    !==================================================
    ! START - Moves robot to home position
    !==================================================
    PROC setup()
        TPWrite "Start command received";
        MoveJ Offs(home,0,0,0),move_speed,z5, tool0,\WObj:=test_reakcji;
        MoveL home, move_speed,fine, tool0,\WObj:=test_reakcji ;
    ENDPROC
ENDMODULE