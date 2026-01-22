MODULE Cymbergaj
    !==================================================
    ! ROBOT DRAWING SYSTEM
    ! Compatible with Python Robot Drawing System
    !==================================================
    
    ! Socket communication variables
    VAR socketdev client_socket;        ! Connection to c++ client
    VAR socketdev server_socket;        ! Server socket listener
    VAR string received_string;         ! Buffer for incoming commands
    
    ! Movement parameters
    CONST speeddata move_speed := v1500;   
    CONST speeddata fast_speed := v5000;   
    CONST zonedata move_zone := z0;  
    CONST num z_up := -10;
    ! variables
    PERS wobjdata current_wobject;
    VAR robtarget target_position;
    
    !==================================================
    ! WAIT FOR CLIENT - Safe connection waiting without RETRY
    !==================================================
    FUNC bool WaitForClient()
        SocketAccept server_socket, client_socket \Time:=10;  ! 10 second timeout
        RETURN TRUE;  ! Successfully accepted a client
        
        ERROR
            IF ERRNO = ERR_SOCK_TIMEOUT THEN
                RETURN FALSE;
            ELSE
                TPWrite "Socket error: " + NumToStr(ERRNO, 0) + " - continuing to wait";
                RETURN FALSE;
            ENDIF
    ENDFUNC
    
    !==================================================
    ! MAIN PROCEDURE - Server initialization and loop
    !==================================================
    PROC main()
        CornerPathWarning FALSE;
        AccSet 100, 100;  
        ! Setup TCP server on port 1025 (matches Python client)
        SocketCreate server_socket;
        SocketBind server_socket, "0.0.0.0", 1025;
        SocketListen server_socket;
        TPWrite "Drawing server ready on port 1025";
        
        ! Main server loop - handles multiple client connections
        WHILE TRUE DO
            ! Try to accept a client connection
            IF WaitForClient() THEN
                TPWrite "Python client connected";
                
                ! Process drawing commands from client
                ProcessClientCommands;
                
                ! Cleanup after client disconnects
                SocketClose client_socket;
                TPWrite "Client disconnected";
            ELSE
                ! No client connected or error - just continue waiting
                WaitTime 1.0;
            ENDIF
        ENDWHILE
        
        SocketClose server_socket;
    ENDPROC
    !==================================================
    ! COMMAND PROCESSING - Main communication loop
    !==================================================
    PROC ProcessClientCommands()
        WHILE TRUE DO
            received_string := "";
            SocketReceive client_socket \Str:=received_string \Time:=40000;
            
            ! Process valid commands (ignore empty strings)
            IF StrLen(received_string) > 0 THEN
                ParseAndExecuteCommand received_string;
            ENDIF
        ENDWHILE
        ERROR
            IF ERRNO = ERR_SOCK_TIMEOUT THEN
                TPWrite "Client timeout - returning to wait for new client";
                SocketClose client_socket;
                RETURN;  ! Return to main loop to wait for new client
            ELSEIF ERRNO = ERR_SOCK_CLOSED THEN
                TPWrite "Socket closed by remote host, cleaning up";
                SocketClose client_socket;
                RETURN;
            ELSE
                TPWrite "Connection error: " + NumToStr(ERRNO, 0);
                TPWrite "Returning to wait for new client connection";
                SocketClose client_socket;
                RETURN; 
            ENDIF
        
    ENDPROC
    
    !==================================================
    ! COMMAND PARSER - Routes commands to handlers
    !==================================================
    PROC ParseAndExecuteCommand(string cmd)
        ! Clean input - remove line endings from Python
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
        
        
        ! Execute movement relative to base_position with precise movement
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
    ! RESPONSE SENDER - Sends status back to Python
    !==================================================
    PROC SendResponse(string msg)
        ! Send the response and terminate with CRLF so the Python client can read a full line (OK\r\n)
        SocketSend client_socket \Str:=msg;
        ! Send CRLF separately to avoid relying on string concatenation semantics
        SocketSend client_socket \Str:="\0D\0A";
    ENDPROC
    
    PROC setup()
        MoveJ Offs(home,0,0,z_up),move_speed,z5,tool1\WObj:=current_wobject;
        MoveL home, move_speed,fine,tool0 \WObj:=current_wobject;
        SendResponse("OK");
    ENDPROC
ENDMODULE