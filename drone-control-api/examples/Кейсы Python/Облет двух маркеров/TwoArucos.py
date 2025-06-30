import time
from drone_control_api import Drone
from drone_control_api.pid import PID
import datetime
import math

ip = "10.42.0.1"
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
    
#Функция поиска маркера
def SearchForArucos():
    client.setDiod(255.0, 0.0, 0.0)
    errors = []
    while len(errors) == 0:
        client.setVelXYYaw(0, 0, -0.5)
        errors = client.getArucos()
        print(errors)
    client.setVelXYYaw(0, 0, 0)

#Функция для отлёта от старого маркера
def AvoidAruco():
    client.setDiod(0.0, 0.0, 255.0)
    client.setVelXYYaw(0, 0, -0.5)
    time.sleep(6)
    client.setVelXYYaw(0, 0, 0)

# Выравнивание относительно маркера
def ArucoRegulation():
    client.setDiod(0.0, 255.0, 0.0)
    pid_pitch= PID(1.5, 0.002, 2.5)
    pid_roll = PID(1.5, 0.002, 2.5)
    pid_yaw = PID(0.6, 0.0, 0.5)

    distance = 0.7

    pitch_error = 0.0
    roll_error = 0.0
    height_error = 0.0
    yaw_error = 0.0

    last_height_regulation_time = 0
    last_yaw_regulation_time = 0



    while True:
        try:
            errors = client.getArucos()
            print(errors)
            if errors:
                roll_error = errors[0]['pose']['position']['x']
                distance_to_marker = errors[0]['pose']['position']['z']
                height_error = errors[0]['pose']['position']['y']

                yaw_error = errors[0]['pose']['orientation']['z']


                pitch_error = distance_to_marker - distance
                
                #print(f"Marker position: roll_error = {roll_error}, distance_error = {pitch_error}, height_error = {height_error}")
                current_time = time.time()

                delta_time_height = current_time - last_height_regulation_time
                delta_time_yaw = current_time - last_yaw_regulation_time


                print(height_error)
                if abs(height_error) > 0.2 and delta_time_height > 5.0:
                    print("height correction")
                    # HeightRegulation(height_error)
                    last_height_regulation_time = time.time()

                
                DefaultRegulation(pitch_error, roll_error, pid_pitch, pid_roll, yaw_error, pid_yaw)
                previous_yaw = yaw_error

                if abs(height_error) < 0.1 and abs(pitch_error) < 0.15 and abs(roll_error) < 0.15 and abs(yaw_error) < 0.15:
                    print("Выравнивание произошло успешно")
                    sucess_flag = 1
                    break

            else: 
                print("Dont see aruco")
                #print(f"Last position marker: roll_error = {roll_error}, distance_error = {pitch_error}, height_error = {height_error}")

                DefaultRegulation(pitch_error, roll_error, pid_pitch, pid_roll, 0, pid_yaw)
                previous_yaw = yaw_error

        except KeyboardInterrupt:
            print("KeyboardInterrupt detected, landing the drone...")
            break


def DefaultRegulation(pitch_error, roll_error, pid_p: PID, pid_r: PID, yaw_error, pid_yaw: PID):

    accuracy = 0.0
    max_pid = 0.8

    if abs(pitch_error) > accuracy:
        pid_p.update_control(pitch_error)
        PID_PITCH = pid_p.get_control()
        PID_PITCH = constrain(PID_PITCH, max_pid)
    else:
        PID_PITCH = 0

    if abs(roll_error) > accuracy:
        pid_r.update_control(roll_error)
        PID_ROLL = pid_r.get_control()  
        PID_ROLL = constrain(PID_ROLL, max_pid)
    else:
        PID_ROLL = 0

    if abs(yaw_error) > 0.2:
        pid_yaw.update_control(yaw_error)
        PID_YAW = - pid_yaw.get_control()
        PID_YAW = constrain(PID_YAW, 0.3)


    else:
        PID_YAW = 0

    print(f"PID Control: PITCH={PID_PITCH}, ROLL={PID_ROLL}, YAW = {PID_YAW}")
    client.setVelXYYaw(PID_PITCH, PID_ROLL, PID_YAW)   

# Выравнивание по высоте
def HeightRegulation(height_error):

    current_height = client.getHeightRange()
    target_height = current_height - height_error

    print(f'current_height is {current_height}')
    print(f'target_height is {target_height}')

    client.setHeight(target_height)

# Функция ограничения величины
def constrain(value, threshold):
    if value > threshold:
        value = threshold
    if value < -threshold:
        value = -threshold
    return value
    
# Функция для учета возможной ошибки инверсии знака оси z аруко маркера
def YawSign(angle, prev_yaw):
    prev_yaw = angle
    if abs(angle - prev_yaw) > 0.3:
        return prev_yaw
    else:
        return angle


def main():
    #Подключаемся
    print("connected?", client.connect(ip, port), "\n")
    #Сбрасываем скорости
    print("VelCorrect", client.setVelXYYaw(0,0,0),"\n")
    #Взлет
    print("takeoff?", client.takeoff(), "\n")
    time.sleep(5)

    #Ищем аруко маркер
    SearchForArucos()
    # Выравнивание относительно маркера
    ArucoRegulation()
    # Отлёт от старого маркера
    AvoidAruco()
    # Ищем следующий маркер
    SearchForArucos()
    # Выравнивание относительно маркера
    ArucoRegulation()
    # Устанавливаем цвет светодиода на красный
    client.setDiod(255.0, 0.0, 0.0)

    #Сбрасываем скорости
    print("VelCorrect", client.setVelXYYaw(0,0,0),"\n")
    #Посадка
    print("boarding?", client.boarding(), "\n")
    
    print("Main function completed successfully!")


client = Drone()
client.AddOnMessangeUtils(OnMes0)

if __name__ == "__main__":
    main()