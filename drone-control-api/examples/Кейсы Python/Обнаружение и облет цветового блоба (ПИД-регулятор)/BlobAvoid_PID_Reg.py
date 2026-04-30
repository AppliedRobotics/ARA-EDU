import time
from drone_control_api import Drone
from drone_control_api.pid import PID
import datetime
import math

ip = "10.42.0.1"
port = "1233"

# v4l2-ctl -c ae_low_limit=50,ae_high_limit=70

def BlobRegulation():
    # Настройка PID регулятора для компоненты ROLL(в данном случае P-регулятор)
    pid_roll = PID(0.0025, 0.0, 0.005)

    # Центральный пиксель изображения, приходящего с камеры
    camera_center_x = 160
    camera_center_y = 120

    # Целевая площадь блоба, при которой дрон приступит к облёту препятствия
    target_area = 45000
    # По дальномеру
    target_distance = 50

    #Минимальная площадь блоба
    threshold_area = 2000

    #Инициализация площади блоба
    blob_area = 0
    roll_error = 0  # Инициализация переменной для избежания ошибки
    ultrasonic_val = None
    while True:
        blobs = client.getBlobs()
        # ultrasonic_val = client.getUltrasonic()[0]['value']
        print(f"Ultrasonic {ultrasonic_val}")
        if isinstance(blobs, str):
            print("Ошибка получения блобов:", blobs)
            break
        if len(blobs) <= 0:
            print("Блобы не обнаружены, обнуляю скорость")
            client.setVelXY(0, 0)
            continue
        else:
            print(f"Обнаружено блобов: {len(blobs)}")
            max_blob = max(blobs, key = lambda b: b['size']['x'] * b['size']['y'])
            blob_area = max_blob['size']['x'] * max_blob['size']['y']
            print(f"Максимальный блоб: {max_blob}")
            if blob_area > threshold_area: 
                roll_error = max_blob["center"]["x"] - camera_center_x
                Regulation(roll_error, pid_roll=pid_roll)
            else:
                client.setVelXY(0, 0)
                print("Нет подходящих блобов. Обнуляю скорость")
        if blob_area >= target_area and abs(roll_error) < 20:
            print(f"Blob area: {blob_area}, Roll_error {roll_error}")
            print("Облёт препятствия")
            avoid()
            break
        
def avoid():
    client.setVelXY(0,0)
    # Обнуляем одометрию
    client.setZeroOdomOpticflow()
    print("Летим вправо")
    time.sleep(1)
    # Летим вправо на 0.5 метра
    client.gotoXYdrone(0.0, -0.5)
    #Обнуляем одометрию
    client.setZeroOdomOpticflow()
    # Летим на 1 метр вперед
    print("Летим вперед")
    time.sleep(1)
    client.gotoXYdrone(0.65, 0.0)
    #Обнуляем одометрию
    client.setZeroOdomOpticflow()
    # Летим влево на 0.5 метра
    print("Летим влево")
    time.sleep(1)
    client.gotoXYdrone(0.0, 0.5)
    
def Regulation(roll_error, pid_roll:PID):
    #Инициализация постоянной скорости PITCH
    x_vel = 0.0
    # Если ошибка ROLL небольшая - летим вперёд с малой скоростью
    if abs(roll_error) < 25:  # Используем abs() для учета отклонений в обе стороны
        x_vel = 0.8
    #Регулировка скорости ROLL
    pid_roll.update_control(roll_error)
    PID_ROLL = pid_roll.get_control()
    #Ограничиваем скорость пороговым значением
    PID_ROLL = constrain_up(PID_ROLL, 1.0)
    print(f"Корректировка PID_ROLL : {PID_ROLL}")
    #Устанавливаем скорость
    client.setVelXY(x_vel, PID_ROLL)

#Функция для ограничения значения переменной
def constrain(value, threshold):
    if value > threshold:
        value = threshold
    if value < -threshold:
        value = -threshold
    return value
def constrain_up(value, threshold):
    new_value = 0.6 * sign(value) + value
    if new_value > threshold:
        new_value = threshold
    if new_value < -threshold:
        new_value = -threshold
    return new_value


def sign(value):
    if value > 0:
        return 1
    elif value < 0:
        return -1
    else:
        return 0
    
client = Drone()


print("connected?", client.connect(ip, port), "\n")
print("VelCorrect", client.setVelXYYaw(0,0,0),"\n")
print("takeoff?", client.takeoff(), "\n")
client.setZeroOdomOpticflow()
time.sleep(7)
print(BlobRegulation())

print("VelCorrect", client.setVelXYYaw(0,0,0),"\n")
print("boarding?", client.boarding(), "\n")
