from drone_control_api import Drone
from drone_control_api.pid import PID
import time
import math

ip = "10.42.0.1"
port = "1233"


def search_aruco(marker_id, calculating_angle=False, shoot=False):
    pid_yaw = PID(0.5, 0.0, 0.0)
    accuracy = 0.15
    client.setVelXYYaw(0.0, 0.0, 0.5)
    search = True
    while search:
        marker_info = client.getArucos()
        if marker_info:
            for marker in marker_info:
                if marker['id'] == marker_id:

                    if calculating_angle:
                        roll_error = marker['pose']['position']['x']
                        pitch_error = marker['pose']['position']['z']
                        yaw_error = math.atan2(roll_error, pitch_error)

                    else:
                        yaw_error = -1 * marker['pose']['orientation']['z']
                    
                    print("yaw_error", yaw_error)

                    pid_yaw.update_control (yaw_error)
                    PID_yaw = pid_yaw.get_control()
                    PID_yaw = constrain(PID_yaw, 0.5)

                    client.setVelXYYaw(0.0, 0.0, PID_yaw)

                    if abs(yaw_error) < accuracy:
                        client.setVelXYYaw(0.0, 0.0, 0.0)
                        print(f"Выравнивание по yaw на маркер {marker_id} завершено")
                        if shoot:
                            client.setShoot(0.15)
                            print("Выстрел")

                        search = False


def aruco_reg():
    pid_pitch = PID(0.5, 0.0, 0.0)
    pid_roll = PID(0.5, 0.0, 0.0)
    accuracy = 0.2
    while True:
        marker_info = client.getArucos()
        if marker_info:
            if marker_info [0]['id'] == 0:
                distance_to_marker = marker_info [0]['pose']['position']['z']
                pitch_error = distance_to_marker - 1
                roll_error = marker_info[0]['pose']['position']['x']

                pid_pitch.update_control(pitch_error)
                PID_pitch = pid_pitch.get_control()
                PID_pitch = constrain (PID_pitch, 0.3)

                pid_roll.update_control(roll_error)
                PID_roll = pid_roll.get_control()
                PID_roll = constrain (PID_roll, 0.3)

                client.setVelXYYaw(PID_pitch, PID_roll, 0.0)

                if abs(pitch_error) < accuracy and abs(roll_error) < accuracy:
                    client.setVelXYYaw(0.0, 0.0, 0.0)
                    print("Выравнивание на маркер с id 0 завершено")
                    break

def marker_priority():
    client.setVelXYYaw(0.0, 0.0, 0.5)
    while True:
        marker_info = client.getArucos()
        if marker_info:
            if len(marker_info) == 3:
                client.setVelXYYaw(0.0, 0.0, 0.0)
                marker_priority_list = [marker['id'] for marker in marker_info]
                marker_priority_list.sort()
                time.sleep(0.5)
                return marker_priority_list


def constrain (value, threshold):
    if value > threshold:
        value = threshold
    if value < -threshold:
        value = -threshold
    return value




client = Drone()

client.connect(ip, port)
client.setVelXYYaw(0.0, 0.0, 0.0)
client.takeoff()
time.sleep(15)
client.setHeight(0.9)
print("Поиск маркера с id 0")
search_aruco(marker_id=0)
print("Выравнивание на маркер с id 0")
aruco_reg()

print("Поиск трех маркеров")
marker_priority_list = marker_priority()
print("Порядок стрельбы по маркерам", marker_priority_list)

for i in marker_priority_list:
    print("Поиск маркера с id", i)
    search_aruco(marker_id=i, calculating_angle=True, shoot=True)

print("КОнец")
client.boarding()
client.disconnect()

