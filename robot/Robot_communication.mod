MODULE Communication
    
    VAR string client_ip;
    VAR num client_port;
    VAR socketdev udp_socket;
    VAR rawbytes received_data;
    VAR num command_code;
    VAR num x_coord;
    VAR num y_coord;
    
    CONST robtarget home:=[[0.0,0.0,0.0],[1.0627E-05,0.996332,0.0855665,-5.29063E-05],[1,0,1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    PROC main()
        ! Create UDP socket on port 1025
        SocketCreate udp_socket \UDP ;
        SocketBind udp_socket, "0.0.0.0", 1025;
        WHILE TRUE DO
            
            SocketReceiveFrom udp_socket \RawData:=received_data, client_ip, client_port;
            ! Unpack data
            UnpackRawBytes received_data, 5, x_coord \Float4;
            UnpackRawBytes received_data, 9, y_coord \Float4;
            
!            TPWrite "Received UDP from " + client_ip + ":" + NumToStr(client_port, 0);
!            TPWrite "X: " + NumToStr(x_coord, 2) + " mm, Y: " + NumToStr(y_coord, 2) + " mm";
            
            !puck_target := Offs(home, x_coord, y_coord, 0);
            puck_target := Offs(home, 108, y_coord, 0);
            new_data_available := TRUE;
        ENDWHILE
    ENDPROC
ENDMODULE