import time 
from drone_control_api import Drone 
from drone_control_api.pid import PID
import datetime 

ip = "10.42.0.1" 
port = "1233" 

def WallFollow():
    pid_pitch= PID(2.5, 0.0, 0.5) 
    distance = 0.5 
    pitch_error = 0.0 
    response_distance = 1.0 
    while True: 
        try: 
            distance_to_wall = client.getUltrasonic()[0]['value'] 
            distance_to_wall /= 100 
            print('distance to wall', distance_to_wall) 
            if distance_to_wall < response_distance : 
                pitch_error = distance_to_wall - distance 
                print('pitch_error', pitch_error) 
                DefaultRegulation(pitch_error, pid_pitch) 
            else: 
                client.setVelXY(1.0, 0) 

        except KeyboardInterrupt: 
            print("KeyboardInterrupt detected, landing the drone...") 
            break 

def DefaultRegulation(pitch_error, pid_pitch: PID): 
    accuracy = 0.1 
    max_pid = 1.0
    max_speed_roll = 0.9
    if abs(pitch_error) > accuracy: 
        pid_pitch.update_control(pitch_error) 
        PID_PITCH = pid_pitch.get_control() 
        PID_PITCH = constrain(PID_PITCH, max_pid) 
    else: 
        PID_PITCH = 0 
    print(f"PID Control: PITCH={PID_PITCH}, ROLL={PID_PITCH }") 
    client.setVelXY(PID_PITCH, max_speed_roll) 

def constrain(value, threshold): 
    if value > threshold: 
        value = threshold 
    if value < -threshold: 
        value = -threshold 
    return value 

client = Drone() 

print("connected?", client.connect(ip, port), "\n") 
print("VelCorrect", client.setVelXYYaw(0,0,0),"\n") 
print("takeoff?", client.takeoff(), "\n") 
time.sleep(4) 
client.setHeight(0.5) 
time.sleep(5) 
print("Wall Follow: ", WallFollow(), "\n") 
print("VelCorrect", client.setVelXYYaw(0,0,0),"\n")
print("boarding?", client.boarding(), "\n")