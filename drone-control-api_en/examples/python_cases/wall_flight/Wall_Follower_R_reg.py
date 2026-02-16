
from drone_control_api import Drone
import time

ip = "192.168.1.197"
port = "1233"

def wall_follow():
    distance = 0.5

    while True:
        distance_to_wall = client.getUltrasonic()[0]['value']
        distance_to_wall /= 100
        print('dist to wall', distance_to_wall)
        pitch_error = distance_to_wall - distance
        print('pitch_error', pitch_error)

        if distance_to_wall > distance:
            client.setDiod(0, 255, 0)
        else:
            client.setDiod(255, 0, 0)

        r_regulator(pitch_error)


def r_regulator(pitch_error):
    max_speed_roll = -0.65
    pitch_speed = sign(pitch_error) * 0.65
    client.setVelXY(pitch_speed, max_speed_roll)  

def sign(value):
    if value >= 0:
        value = 1
    else:
        value = -1
    return value 

client = Drone()

print("connected?", client.connect(ip, port), "\n")
print("VelCorrect", client.setVelXYYaw(0,0,0),"\n")
print("takeoff?", client.takeoff(), "\n")
time.sleep(5)

wall_follow()

