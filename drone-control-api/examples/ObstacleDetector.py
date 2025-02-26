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




def ObstacleDetector():
    pid_pitch= PID(2.5, 0.0, 0.0)
    ###дистанция до стены
    distance = 0.8

    pitch_error = 0.0
#скорость движения вдоль стены при необходимости
    max_roll_speed = 1.0

    while True:
        try:
            #получение дистанции до стены
            distance_to_wall = client.getUltrasonic()[0]['value']
            distance_to_wall /= 100
            print('dist to wall', distance_to_wall)
            #расчет ошибки расстояния до стены
            pitch_error = distance_to_wall - distance
            print('pitch_error', pitch_error)
            #если дистанция до стены меньше 1.5 и больше дистанции до стены, то включаем зеленый свет и звук
            if distance_to_wall < 1.5 and distance_to_wall > distance:
                client.setDiod(0.0, 255.0, 0.0)
                client.setBeeper(255.0, 10.0)
                roll = 0.0 #установить max_roll_speed если нужно лететь вдоль стены
            #если дистанция до стены меньше минимальной дистанции до стены, то включаем красный свет и звук
            elif distance_to_wall < distance and distance_to_wall > 0.0:
                client.setDiod(255.0, 0.0, 0.0)
                client.setBeeper(255.0, 10.0)
                roll = 0.0 #max_roll_speed если нужно лететь боком
            #если дистанция до стены больше минимальной дистанции до стены, то выключаем светодиоды и звук
            else:
                client.setDiod(0.0, 0.0, 0.0)
                client.setBeeper(0.0, 0.0)
                roll = 0.0
            #регулирование удержания на заданном расстоянии
            DefaultRegulation(pitch_error, pid_pitch, roll) 


        except KeyboardInterrupt:
            print("KeyboardInterrupt detected, landing the drone...")
            client.setDiod(0.0, 0.0, 0.0)
            client.setBeeper(0.0, 0.0)
            break

def DefaultRegulation(pitch_error, pid_p: PID, max_roll_speed):

    accuracy = 0.1
    max_pid = 1.5
    #если ошибка больше заданной то регулируем движение
    if abs(pitch_error) > accuracy:
        #расчет ошибки регулирования
        pid_p.update_control(pitch_error)
        #получение значения регулирования
        PID_PITCH = pid_p.get_control()
        #ограничение значения регулирования
        PID_PITCH = constrain(PID_PITCH, max_pid)
    else:
        PID_PITCH = 0


    print(f"PID Control: PITCH={PID_PITCH}, ROLL={max_roll_speed}")

    client.setVelXY(PID_PITCH, max_roll_speed)  

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

time.sleep(5)
client.setHeight(0.5)

print("Wall Follow: ", ObstacleDetector(), "\n")

client.setDiod(0.0, 0.0, 0.0)
client.setBeeper(0.0, 0.0)
print("VelCorrect", client.setVelXYYaw(0,0,0),"\n")
print("boarding?", client.boarding(), "\n")
