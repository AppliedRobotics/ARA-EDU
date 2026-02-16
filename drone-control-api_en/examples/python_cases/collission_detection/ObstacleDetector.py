import time
from drone_control_api import Drone
from drone_control_api import pid

ip = "192.168.1.197"
port = "1233"



def obstacle_detector():
    distance = 0.7

    while True:
        distance_to_wall = client.getUltrasonic()[0]['value']
        distance_to_wall /= 100
        print('dist to wall', distance_to_wall)

        if distance_to_wall > distance:
            client.setDiod(0, 255, 0)
            client.setVelXY(0.9, 0)
            client.setBeeper(0, 0)

        else:
            client.setDiod(255, 0, 0.0)
            client.setBeeper(50, 1000)
            client.setVelXY(0, 0)
            break



client = Drone()


print("connected?", client.connect(ip, port), "\n")
client.setDiod(0,0,0)
print("VelCorrect", client.setVelXYYaw(0,0,0),"\n")
print("takeoff?", client.takeoff(), "\n")
time.sleep(5)
client.setHeight(0.5)

obstacle_detector()
time.sleep(3)

print("VelCorrect", client.setVelXYYaw(0,0,0),"\n")
print("boarding?", client.boarding(), "\n")

time.sleep(3)
client.setDiod(0.0, 0.0, 0.0)
client.setBeeper(0.0, 0.0)
