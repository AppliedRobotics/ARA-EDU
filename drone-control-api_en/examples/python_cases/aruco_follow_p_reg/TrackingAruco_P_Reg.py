from drone_control_api import Drone
from drone_control_api.pid import PID  
import time

ip = "10.42.0.1"
port = "1233"

# Функция поиска маркера
def search_aruco (yaw_vel):
    client.setDiod(255, 0, 0)
    client.setVelXYYaw(0, 0, yaw_vel)
    aruco_state = True
    while aruco_state: 
        errors = client.getArucos()
        if errors:
            aruco_state = False
            client.setDiod(0, 255, 0)
            client.setVelXYYaw(0, 0, 0)

# Функция расчета ошибок
def tracking_aruco():
    pitch_error = 0 
    roll_error = 0
    yaw_error = 0
    distance = 1
    accuracy_height = 0.15
    while True:
        errors = client.getArucos()
        if errors:
            client.setDiod(0, 255, 0)
            distance_to_marker = errors [0]['pose']['position']['z']
            pitch_error = distance_to_marker - distance
            roll_error = errors[0]['pose']['position']['x']
            height_error = errors[0]['pose']['position']['y']
            yaw_error = -1 * errors[0]['pose']['orientation']['z']
            print(f"Дистанция до маркера: {distance_to_marker}")
            
            if abs(height_error) > accuracy_height:
                height_regulation(height_error)

            aruco_regulation (pitch_error, roll_error, yaw_error)
        else:
            client.setDiod(255, 0, 0)
            aruco_regulation (pitch_error, roll_error, 0)
            print (f"Маркер потерян! Поиск:")
            print (f"pitch_error = {pitch_error}, roll_error = {roll_error}")

# Выравнивание по высоте 
def height_regulation(height_error):
    current_height = client.getHeightRange() 
    new_height = current_height - height_error
    print(f"Новая высота {new_height}")
    client.setHeight(new_height)

# Выравнивание относительно маркера
def aruco_regulation(pitch_error, roll_error, yaw_error):
    PID_pitch = pitch_error * 1.5
    PID_pitch = constrain (PID_pitch, 1)
    PID_roll = roll_error * 1.5
    PID_roll = constrain (PID_roll, 1)
    PID_yaw = constrain (yaw_error, 0.3)
    print(f"PID_pitch={PID_pitch}, PID_roll={PID_roll}, PID_yaw={PID_yaw}")
    client.setVelXYYaw(PID_pitch, PID_roll, PID_yaw)

# Функция ограничения величины
def constrain (value, threshold):
    if value > threshold:
        value = threshold
    if value < -threshold:
        value = -threshold
    return value


client = Drone()

# Подключаемся
print("connected?", client.connect(ip, port), "\n")
# Сбрасываем скорости
print("VelCorrect", client.setVelXYYaw(0,0,0),"\n")
# Взлет
print("takeoff?", client.takeoff(), "\n")
time.sleep(7)

# Ищем маркер
search_aruco(0.5)

# Выравнивание относительно маркера
tracking_aruco()