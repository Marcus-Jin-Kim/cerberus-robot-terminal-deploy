# Input the library for base control
from base_ctrl import BaseController
import json
# Function for Detecting Raspberry Pi
# Function for Detecting Raspberry Pi
# def is_raspberry_pi5():
#     with open('/proc/cpuinfo', 'r') as file:
#         for line in file:
#             if 'Model' in line:
#                 if 'Raspberry Pi 5' in line:
#                     return True
#                 else:
#                     return False

# # Determine the GPIO Serial Device Name Based on the Raspberry Pi Model
# if is_raspberry_pi5():
#     base = BaseController('/dev/ttyAMA0', 115200)
# else:
#     base = BaseController('/dev/serial0', 115200)

#The loop will exit and display the feedback information 
# after receiving the first complete JSON message with a "T" value of 1001. 
# The feedback information includes the current wheel speed, 
# IMU data, 
# gimbal angle (if installed), 
# arm angle (if installed), 
# power voltage, 
# and other data.
# {'T': 1001, 'L': 0, 'R': 0, 
# 'ax': 90, 'ay': -402, 'az': 8206, 
# 'gx': -12, 'gy': 0, 'gz': 11, 
# 'mx': -684, 'my': 377, 'mz': 976, 
# 'odl': 0, 
# 'odr': 0, 
# 'v': 1217} volt? power?
# WavesharePi/ugv_rpi/tutorial_en/08 Microcontroller JSON Command Set.ipynb


def print_feedback(t=5,i=0.01):
    if t <= 0.1:
        t = 0.1
    t_end = time.time() + t
    while time.time() < t_end:
        if base.rl.s.in_waiting > 0:
            t_raw = base.rl.readline()
            try:
                t_recv = t_raw.decode('utf-8').strip()

                if t_recv and t_recv.startswith('{') and t_recv.endswith('}'):
                
                    data = json.loads(t_recv)
                    if 'T' in data:
                        # print(data)
                        if data["T"] == 1001:
                            # print("Feedback received:")
                            print(f"{data['pan']},{data['tilt']}")
                            pass
            except Exception as e:
                print(f"[CB][print_feedback] error: {e}")
                pass    

            # finally:
            #     base.rl.clear_buffer()
            
        # f = base.feedback_data()
        # # print(f)
        # if f is not None:
        #     if f["T"] == 1001:
        #         # print("Feedback received:")
        #         print(f"{f['pan']},{f['tilt']}")
        
        # time.sleep(i)

base = BaseController('/dev/ttyAMA0', 115200)
# #### CMD_MM_TYPE_SET
# - {"T":900,"main":2,"module":0}
# - Set the mainType(chassis type) and the module type for chassis.
# > main:  
# > 1 - RaspRover  
# > 2 - UGV Rover  
# > 3 - UGV Beast  
# > module:  
# > 0 - None  
# > 1 - RoArm  
# > 2 - Pan-Tilt  

base.send_command({"T": 900, "main": 3, "module": 2})
while True:
    if base.rl.s.in_waiting > 0:
        try:
            t_raw = base.rl.readline()
            t_recv = t_raw.decode('utf-8').strip()
            if t_recv and t_recv.startswith('{') and t_recv.endswith('}'):                
                data = json.loads(t_recv)
                if 'T' in data:
                    # print(data)
                    if data["T"] == 1001:
                        # print("Feedback received:")
                        print(f"{data['pan']},{data['tilt']},{data['ax']},{data['ay']},{data['az']}")
        except Exception as e:
            print(f"[CB][main loop] error: {e}")
            pass



# base.gimbal_ctrl(-40, 0, 0, 0)
# base.lights_ctrl(255, 255)
# print("gimbal to -40,0,0,0 and lights to 255,255")
# print(base.feedback_data())
# print_feedback()



# base.gimbal_ctrl(0, 15, 0, 0)
# base.lights_ctrl(0, 0)
# print("gimbal to 0,15,0,0 and lights to 0,0")

# print(base.feedback_data())
# print_feedback()


# base.gimbal_ctrl(0, -15, 0, 0)
# base.lights_ctrl(0, 0)
# print("gimbal to 0,-15,0,0 and lights to 0,0")

# print(base.feedback_data())
# print_feedback()



# base.gimbal_ctrl(0, 0, 0, 0)
# print("gimbal to 0,0,0,0 and lights to 0,0")

# print(base.feedback_data())
# print_feedback()
