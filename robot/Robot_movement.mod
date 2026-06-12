MODULE Cymbergaj
    !==================================================
    ! AIR HOCKEY ROBOT SYSTEM (UDP)
    !==================================================
    ! From socket
    VAR string client_ip;
    VAR num client_port;
    VAR socketdev udp_socket;
    VAR rawbytes received_data;
    VAR num command_code;
    VAR num x_coord;
    VAR num y_coord;
    
    
    VAR num new_angle;

    
    ! Movement parameters - optimized for speed
    CONST speeddata move_speed := v2000;
    CONST speeddata fast_speed := v7000;
    CONST zonedata move_zone := z50;
    ! variables
    PERS wobjdata current_wobject;
    VAR robtarget target_position;
    CONST robtarget home:=[[0.0,0.0,0.0],[0.024379,0.995874,0.0857285,-0.0170864],[0,-1,1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,0]];
    VAR bool puck_valiable:=FALSE;
    VAR bool flip:=TRUE;
    
    !==================================================
    ! move and stuff if point recieved
    !==================================================
    PROC main()
        AccSet 100, 100;
        SocketCreate udp_socket \UDP ;
        SocketBind udp_socket, "10.25.74.172", 1025;
        GripLoad load0;
        WHILE TRUE DO
        
            SocketReceiveFrom udp_socket \RawData:=received_data, client_ip, client_port;
            UnpackRawBytes received_data, 1, y_coord \Float4;
            
            !puck_target := Offs(home, x_coord, y_coord, 0);
            jpos30.robax.rax_1 := y_coord;
            MoveAbsJ jpos30, fast_speed, move_zone, paddle; 
!            IF flip THEN
!                MoveAbsJ jpos30, fast_speed, move_zone, paddle; 
!                flip:=FALSE;
!            ELSE 
!                MoveAbsJ jpos40, fast_speed, move_zone, paddle;
!                flip:=TRUE;
!            ENDIF
!            WaitTime 0.001;
        ENDWHILE
    ENDPROC
ENDMODULE
