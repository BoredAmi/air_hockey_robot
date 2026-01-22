MODULE Cymbergaj
    !==================================================
    ! AIR HOCKEY ROBOT SYSTEM (UDP)
    !==================================================
    
    ! Socket communication variables
    VAR socketdev udp_socket; 
    VAR string received_string;
    
    ! Movement parameters - optimized for speed
    CONST speeddata move_speed := v2000; 
    CONST speeddata fast_speed := v7000; 
    CONST zonedata move_zone := z0;
    CONST num z_up := -10;
    ! variables
    PERS wobjdata current_wobject;
    VAR robtarget target_position;
    
    !==================================================
    ! MAIN PROCEDURE - UDP server initialization
    !==================================================
    PROC main()
        CornerPathWarning FALSE;
        AccSet 100, 100;

        ! Create UDP socket on port 1025
        SocketCreate udp_socket;
        SocketBind udp_socket, "0.0.0.0", 1025;

        TPWrite "UDP Air Hockey server ready on port 1025";

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
            received_string := "";
            ! Receive packet
            SocketReceiveFrom udp_socket \Str:=received_string, client_ip, client_port \Time:=0.1;
            
            ! Process valid commands
            IF StrLen(received_string) > 0 THEN
                ParseAndExecuteCommand received_string;
            ENDIF
        ENDWHILE
        
        ERROR
            IF ERRNO = ERR_SOCK_TIMEOUT THEN
                ! No message received, continue loop
                RETURN;
            ELSE
                TPWrite "UDP error: " + NumToStr(ERRNO, 0);
                RETURN;
            ENDIF
    ENDPROC
    
    !==================================================
    ! COMMAND PARSER - Routes commands to handlers
    !==================================================
    PROC ParseAndExecuteCommand(string cmd)
        ! Clean input - remove line endings
        cmd := StrMap(cmd, "\0A\0D", "");
        ! Route commands to appropriate handlers
        TEST cmd
        DEFAULT:
            IF StrMatch(cmd, 1, "STOP") = 1 THEN
                HandleStopCommand;
            ELSEIF StrMatch(cmd, 1, "MOVE,") = 1 THEN
                HandleMoveCommand cmd;
            ELSEIF StrMatch(cmd, 1, "START") = 1 THEN
                setup;
            ELSE
                SendResponse("ERROR: Unknown command");
            ENDIF
        ENDTEST
        
    ERROR
        TPWrite "Command error: " ;
        SendResponse("ERROR: ");
    ENDPROC
    

    !==================================================
    ! MOVEMENT HANDLER - Processes MOVE,X,Y commands
    !==================================================
    PROC HandleMoveCommand(string cmd)
        VAR num x_coord;    ! Parsed X coordinate (mm)
        VAR num y_coord;    ! Parsed Y coordinate (mm)
        VAR num pos1;       ! First comma position
        VAR num pos2;       ! Second comma position
        
        ! Parse command format: "MOVE,X,Y"
        pos1 := StrFind(cmd, 1, ",");
        pos2 := StrFind(cmd, pos1 + 1, ",");
        
        ! Validate command structure
        IF pos1 = 0 OR pos2 = 0 THEN
            SendResponse("ERROR: Format should be MOVE,X,Y");
            RETURN;
        ENDIF
        
        ! Extract and validate numeric coordinates
        IF NOT(StrToVal(StrPart(cmd, pos1 + 1, pos2 - pos1 - 1), x_coord) AND
               StrToVal(StrPart(cmd, pos2 + 1, StrLen(cmd) - pos2), y_coord)) THEN
            SendResponse("ERROR: Invalid coordinates");
            RETURN;
        ENDIF
        
        
        ! Execute movement relative to base_position
        target_position := offs(home, x_coord, y_coord, 0);
        MoveL target_position, fast_speed, move_zone, tool1 \WObj:=current_wobject;
        
        SendResponse("OK");
    ENDPROC
    
    !==================================================
    ! STOP - Returns robot to safe position
    !==================================================
    PROC HandleStopCommand()
        TPWrite "Stop command received";
        ! Return to base position safely with smooth movement
        target_position := home;
        MoveL Offs(target_position, 0, 0, z_up), fast_speed, move_zone, tool1 \WObj:=current_wobject;
        SendResponse("STOPPED");
    ENDPROC
    
    !==================================================
    ! RESPONSE SENDER - Optional UDP response (disabled for speed)
    !==================================================
    PROC SendResponse(string msg)
        ! For UDP speed, we skip responses to minimize latency
        ! Uncomment below if you need confirmation:
        ! SocketSendTo udp_socket \Str:=msg + "\0D\0A" \RemoteAddress:=client_ip \RemotePort:=client_port;
    ENDPROC
    
    PROC setup()
        MoveJ Offs(home,0,0,z_up),move_speed,z5,tool1\WObj:=current_wobject;
        MoveL home, move_speed,fine,tool0 \WObj:=current_wobject;
        ! No response for UDP speed
    ENDPROC
ENDMODULE