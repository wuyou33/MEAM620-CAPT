function pos = update_pos(robot_struct,t_c,t_f)

pos = (1-(t_c-robot_struct.t_0)/(t_f-robot_struct.t_0))*robot_struct.start...
    + (t_c-robot_struct.t_0)/(t_f-robot_struct.t_0)*robot_struct.goal;
        