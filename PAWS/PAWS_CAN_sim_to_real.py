#!/usr/bin/env python
import can
import time
import os 
#import rospy
import math
import signal
import sys  
import matplotlib.pyplot as plt
import numpy as np

#from sensor_msgs.msg import JointState

os.system('sudo ifconfig can0 down')
os.system('sudo ip link set can0 type can bitrate 1000000')
os.system('sudo ifconfig can0 txqueuelen 100000')
os.system('sudo ifconfig can0 up')

current_positions = [0.0] * 12  # Initialize with zeros
current_velocities = [0.0] * 12  # Initialize with zeros



def sigint_handler(sig, frame):   # This function will be called when Ctrl+C is pressed  
    print("Ctrl+C detected. Shutting down...")
    #plt.plot(range(len(position)),position)
    plt.show()
    time.sleep(0.5)
    shutdown_motor()
    sys.exit(0)

def shutdown_motor():   
    try:
        bus = can.Bus(interface='socketcan', channel='can0', bitrate=100000)
        for i in range(12): 
            motor_id = 0x141 + i
            msg_sd = can.Message(arbitration_id=motor_id, data=[0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], is_extended_id=False)
            bus.send(msg_sd)
            time.sleep(0.1)
            print("frame", msg_sd)
        print("Loop interrupted. Stopping...")
    finally:
        bus.shutdown()

def joint_state_callback(data):
    global current_positions 
    global current_velocities
    current_positions = list(data.position)  # Update the current positions
    current_velocities = data.velocity  # Update the current velocities

def radians_to_control_value(radians):
    # Conversion factor: 1 radian = 5729.57795 control value (0.01-degree)
    pos_in_deg = (radians * (180.0 / math.pi))
    print(pos_in_deg)
    control_value = int(pos_in_deg * 100)   # 0.01degree/LSB
    print(control_value)
    return control_value

def rad_per_sec_to_control_value(rad_per_sec):
    # Conversion factor: 1 radian/second = 100 control value (1dps/LSB)
    vel_in_deg = rad_per_sec * (180.0 / math.pi)
    control_value = int(vel_in_deg)   # 1dps/LSB
    print(control_value)
    return control_value


def gen_can_msg(position, motor_id):
    bus = can.Bus(interface='socketcan', channel='can0', bitrate=100000)
    control_value_position = radians_to_control_value(position)

    msg = can.Message(arbitration_id=0x140+motor_id, data=[0xa4, 0x00, 0x50,0x00,
                                                        control_value_position  & 0xFF, (control_value_position >> 8) & 0xFF,
                                                        (control_value_position >> 16) & 0xFF, (control_value_position >> 24) & 0xFF], is_extended_id=False)
    print("frame", msg)
    print("pos", position* (180.0 / math.pi))
    bus.send(msg)


def main():
    #rospy.init_node('CAN_send', anonymous=True)
    #rospy.Subscriber('/joint_states', JointState, joint_state_callback) 

    data = np.loadtxt('bezier_fixed_angles_trot.csv', delimiter=',', skiprows=1)  # Assuming headers are present
    #print("DATA", data)
    while True:
        for j in range (1, 800) :
                    
                offset = 12*j
                FR_Hip_CAN_Index = 0 + offset
                FR_Knee_CAN_Index = 1 + offset
                FR_Foot_CAN_Index = 2 + offset

                FL_Hip_CAN_Index = 3 + offset
                FL_Knee_CAN_Index = 4 + offset
                FL_Foot_CAN_Index = 5 + offset

                RR_Hip_CAN_Index = 6 + offset
                RR_Knee_CAN_Index = 7 + offset
                RR_Foot_CAN_Index = 8 + offset

                RL_Hip_CAN_Index = 9 + offset
                RL_Knee_CAN_Index = 10 + offset
                RL_Foot_CAN_Index = 11 + offset

                FR_Hip_CAN_Angle = data[FR_Hip_CAN_Index]
                FR_Knee_CAN_Angle = data[FR_Knee_CAN_Index]
                FR_Foot_CAN_Angle = data[FR_Foot_CAN_Index]

                FL_Hip_CAN_Angle = data[FL_Hip_CAN_Index]
                FL_Knee_CAN_Angle = data[FL_Knee_CAN_Index]
                FL_Foot_CAN_Angle = data[FL_Foot_CAN_Index]

                RR_Hip_CAN_Angle = data[RR_Hip_CAN_Index]
                RR_Knee_CAN_Angle = data[RR_Knee_CAN_Index]
                RR_Foot_CAN_Angle = data[RR_Foot_CAN_Index]

                RL_Hip_CAN_Angle = data[RL_Hip_CAN_Index]
                RL_Knee_CAN_Angle = data[RL_Knee_CAN_Index]
                RL_Foot_CAN_Angle = data[RL_Foot_CAN_Index]

              
                print("i", j)
                print("FR_Hip_CAN_Angle", FR_Hip_CAN_Angle*(180/math.pi))
                print("FR_Knee_CAN_Angle", FR_Knee_CAN_Angle*(180/math.pi))
                print("FR_Foot_CAN_Angle", FR_Foot_CAN_Angle*(180/math.pi))  
                
                print("FL_Hip_CAN_Angle", FL_Hip_CAN_Angle*(180/math.pi))
                print("FL_Knee_CAN_Angle", FL_Knee_CAN_Angle*(180/math.pi))
                print("FL_Foot_CAN_Angle", FL_Foot_CAN_Angle*(180/math.pi))  



                gen_can_msg(FR_Hip_CAN_Angle, 7)
                gen_can_msg(FR_Knee_CAN_Angle, 8)
                gen_can_msg( -FR_Foot_CAN_Angle, 9)

                gen_can_msg(FL_Hip_CAN_Angle, 10)
                gen_can_msg( -FL_Knee_CAN_Angle, 11)
                gen_can_msg(FL_Foot_CAN_Angle, 12)

                gen_can_msg(RR_Hip_CAN_Angle, 4)
                gen_can_msg(-RR_Knee_CAN_Angle, 5)
                gen_can_msg( RR_Foot_CAN_Angle, 6)

                gen_can_msg(RL_Hip_CAN_Angle, 1)
                gen_can_msg(-RL_Knee_CAN_Angle, 2)
                gen_can_msg(RL_Foot_CAN_Angle, 3)

                time.sleep(0.001)
               




if __name__ == "__main__":
    try:
        signal.signal(signal.SIGINT, sigint_handler)
                
        main()


    except KeyboardInterrupt:
        pass
