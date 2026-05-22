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
    CONST zonedata move_zone := z5;
    ! variables
    PERS wobjdata current_wobject;
    VAR robtarget target_position;
    CONST robtarget home:=[[0.0,0.0,0.0],[1.0627E-05,0.996332,0.0855665,-5.29063E-05],[1,0,1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    VAR bool last_dance:=TRUE;
    VAR bool puck_valiable:=FALSE;
    
    !==================================================
    ! MAIN PROCEDURE - UDP server initialization
    !==================================================
    PROC main()
        AccSet 100, 100;
        IF new_data_available THEN
            ! React immediately! 
            ! Use a massive zone so the robot doesn't stutter
            MoveL puck_target, fast_speed, move_zone, paddle \WObj:=table;
            new_data_available := FALSE;
            puck_valiable := TRUE;
!        ELSEIF last_dance AND puck_valiable THEN
!            ! Optional: Do the 'Dance' here to keep motors engaged
!            MoveL Offs(puck_target,0,20,0), v50, move_zone, paddle \WObj:=table;
!            last_dance:=FALSE;
!        ELSEIF puck_valiable  AND (NOT last_dance) THEN
!            MoveL Offs(puck_target,0,-20,0), v50, move_zone, paddle \WObj:=table;
!            last_dance:=TRUE;
        ENDIF
    ENDPROC
ENDMODULE
