from drone_control_api import Drone
from drone_control_api.pid import PID  
import time

ip = "10.42.0.1"
client = Drone()

# Функция расчета ошибок
def tracking_aruco():
    pitch_error = 0 
    roll_error = 0
    distance = 1
    accuracy_height = 0.2
    while True:
        errors = client.get_arucos()
        print(errors)
        if type(errors) is not str:
            distance_to_marker = errors [0]['pose']['position']['z']
            pitch_error = distance_to_marker - distance
            roll_error = errors[0]['pose']['position']['x']
            height_error = errors[0]['pose']['position']['y']
            yaw_error = -1 * errors[0]['pose']['orientation']['z']
            print(f"Дистанция до маркера: {distance_to_marker}")
            
            if abs(height_error) > accuracy_height:
                height_regulation(height_error)

            aruco_regulation (pitch_error, roll_error)
        else:
            aruco_regulation (pitch_error, roll_error)
            print (f"Маркер потерян! Поиск:")
            print (f"pitch_error = {pitch_error}, roll_error = {roll_error}")

# Выравнивание по высоте 
def height_regulation(height_error):
    current_height = client.get_height_rangefinder() 
    new_height = current_height - height_error * 0.7
    print(f"Новая высота {new_height}")
    client.set_height(new_height)

# Выравнивание относительно маркера
def aruco_regulation(pitch_error, roll_error):
    PID_pitch = pitch_error * 1.5
    PID_pitch = constrain (PID_pitch, 1)
    PID_roll = roll_error * 1.5
    PID_roll = constrain (PID_roll, 1)
    print(f"PID_pitch={PID_pitch}, PID_roll={PID_roll}")
    client.set_vel_xy_yaw(PID_pitch, PID_roll, 0)

# Функция ограничения величины
def constrain (value, threshold):
    if value > threshold:
        value = threshold
    if value < -threshold:
        value = -threshold
    return value

def main():
    # Подключаемся
    print(client.connect(ip, reset_state=True))
    # Сбрасываем скорости
    client.set_vel_xy_yaw(0.0, 0.0, 0.0)
    # Взлет
    client.takeoff()
    print("Взлет завершен")
    # Выравнивание относительно маркера
    tracking_aruco()

main()


