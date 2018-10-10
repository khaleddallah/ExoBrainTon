[ control info ]
control_cycle = 8   # milliseconds

[ port info ]
# PORT NAME  | BAUDRATE | DEFAULT JOINT
/dev/ttyUSB1 | 1000000  | l_leg_hip_y
/dev/ttyUSB0 | 1000000  | r_leg_hip_y
#/dev/ttyUSB2 | 2000000  | r_leg_hip_y
#/dev/ttyUSB3 | 2000000  | l_leg_hip_y

[ device info ]
# TYPE    | PORT NAME    | ID  | MODEL          | PROTOCOL | DEV NAME     | BULK READ ITEMS
#dynamixel | /dev/ttyUSB0 | 1   | H54-100-S500-R | 2.0      | r_arm_sh_p1  | present_position, present_voltage
#dynamixel | /dev/ttyUSB1 | 2   | H54-100-S500-R | 2.0      | l_arm_sh_p1  | present_position, present_voltage
#dynamixel | /dev/ttyUSB0 | 3   | H54-100-S500-R | 2.0      | r_arm_sh_r   | present_position, present_voltage
#dynamixel | /dev/ttyUSB1 | 4   | H54-100-S500-R | 2.0      | l_arm_sh_r   | present_position, present_voltage
#dynamixel | /dev/ttyUSB0 | 5   | H54-100-S500-R | 2.0      | r_arm_sh_p2  | present_position, present_voltage
#dynamixel | /dev/ttyUSB1 | 6   | H54-100-S500-R | 2.0      | l_arm_sh_p2  | present_position, present_voltage
#dynamixel | /dev/ttyUSB0 | 7   | H54-100-S500-R | 2.0      | r_arm_el_y   | present_position, present_voltage
#dynamixel | /dev/ttyUSB1 | 8   | H54-100-S500-R | 2.0      | l_arm_el_y   | present_position, present_voltage
#dynamixel | /dev/ttyUSB0 | 9   | H42-20-S300-R  | 2.0      | r_arm_wr_r   | present_position, present_voltage
#dynamixel | /dev/ttyUSB1 | 10  | H42-20-S300-R  | 2.0      | l_arm_wr_r   | present_position, present_voltage
#dynamixel | /dev/ttyUSB0 | 11  | H42-20-S300-R  | 2.0      | r_arm_wr_y   | present_position, present_voltage
#dynamixel | /dev/ttyUSB1 | 12  | H42-20-S300-R  | 2.0      | l_arm_wr_y   | present_position, present_voltage
#dynamixel | /dev/ttyUSB0 | 13  | H42-20-S300-R  | 2.0      | r_arm_wr_p   | present_position, present_voltage
#dynamixel | /dev/ttyUSB1 | 14  | H42-20-S300-R  | 2.0      | l_arm_wr_p   | present_position, present_voltage



dynamixel | /dev/ttyUSB1 | 7   | MX-64 | 1.0      | l_leg_hip_y  | present_position, present_voltage
dynamixel | /dev/ttyUSB1 | 8   | MX-64 | 1.0      | l_leg_hip_r  | present_position, present_voltage
dynamixel | /dev/ttyUSB1 | 9   | MX-64 | 1.0      | l_leg_hip_p  | present_position, present_voltage
dynamixel | /dev/ttyUSB1 | 10  | MX-64 | 1.0      | l_leg_kn_p   | present_position, present_voltage
dynamixel | /dev/ttyUSB1 | 11  | MX-64 | 1.0      | l_leg_an_p   | present_position, present_voltage
dynamixel | /dev/ttyUSB1 | 12  | MX-64 | 1.0      | l_leg_an_r   | present_position, present_voltage




dynamixel | /dev/ttyUSB0 | 1   | MX-64 | 1.0      | r_leg_hip_y  | present_position, present_voltage
dynamixel | /dev/ttyUSB0 | 2   | MX-64 | 1.0      | r_leg_hip_r  | present_position, present_voltage
dynamixel | /dev/ttyUSB0 | 3   | MX-64 | 1.0      | r_leg_hip_p  | present_position, present_voltage
dynamixel | /dev/ttyUSB0 | 4   | MX-64 | 1.0      | r_leg_kn_p   | present_position, present_voltage
dynamixel | /dev/ttyUSB0 | 5   | MX-64 | 1.0      | r_leg_an_p   | present_position, present_voltage
dynamixel | /dev/ttyUSB0 | 6   | MX-64 | 1.0      | r_leg_an_r   | present_position, present_voltage

