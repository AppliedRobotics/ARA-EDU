import time
from drone_control_api import Drone
from drone_control_api.pid import PID
import datetime
import math

ip = "trackingcam3.local"
port = "1233"

last_time = None



def OnMes0(mes):
    global last_time
    current_time = datetime.datetime.now()
    if last_time is not None:
        delta_time = (current_time - last_time).total_seconds()
    else:
        delta_time = None
    
    last_time = current_time
    
    if delta_time is not None:
        log_message = f"Timestamp: {current_time}, Message: {mes}, Delta Time: {delta_time:.2f} seconds\n"
    else:
        log_message = f"Timestamp: {current_time}, Message: {mes}, Delta Time: N/A\n"

    with open("messages_log.txt", "a") as file:
        file.write(log_message)
    # print(log_message)




def WallFollow():
    pid_pitch= PID(2.5, 0.0, 0.0)

    distance = 0.5

    pitch_error = 0.0

    while True:
        try:
            distance_to_wall = client.getUltrasonic()[0]['value']
            distance_to_wall /= 100
            print('dist to wall', distance_to_wall)
            pitch_error = distance_to_wall - distance
            print('pitch_error', pitch_error)
            DefaultRegulation(pitch_error, pid_pitch)

        except KeyboardInterrupt:
            print("KeyboardInterrupt detected, landing the drone...")
            break

def DefaultRegulation(pitch_error, pid_p: PID):

    accuracy = 0.1
    max_pid = 1.5
    max_speed_roll = 1.0

    if abs(pitch_error) > accuracy:
        pid_p.update_control(pitch_error)
        PID_PITCH = pid_p.get_control()
        PID_PITCH = constrain(PID_PITCH, max_pid)
    else:
        PID_PITCH = 0

    print(f"PID Control: PITCH={PID_PITCH}, ROLL={0}")

    client.setVelXY(PID_PITCH, max_speed_roll)      #client.setVelXY(PID_PITCH, 0 ) без движения боком

def constrain(value, threshold):
    if value > threshold:
        value = threshold
    if value < -threshold:
        value = -threshold
    return value 




client = Drone()
client.AddOnMessangeUtils(OnMes0)


print("connected?", client.connect(ip, port), "\n")
print("VelCorrect", client.setVelXYYaw(0,0,0),"\n")
print("takeoff?", client.takeoff(), "\n")
time.sleep(4)
client.setHeight(0.5)
time.sleep(5)
print("Wall Follow: ", WallFollow(), "\n")

print("VelCorrect", client.setVelXYYaw(0,0,0),"\n")
print("boarding?", client.boarding(), "\n")
