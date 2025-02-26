# Библиотеки для работы со временем
import time
from datetime import datetime

# Класс для работы с API дрона
from drone_control_api import Drone   

# Импортируем класс PID
from drone_control_api.pid import PID

# Задаем IP и порт для подключения к дрону
ip = "trackingcam3.local"
port = "1233"

# Переменная для хранения времени последнего сообщения
last_time = None

# Функция посадки дрона на аруко-маркер
def Aruco_boarding():
    # Время начала отсчёта
    time_begin = datetime.now()
    # Инициализируем переменную для текущего времени
    time_now = time_begin
    # Находим их разность
    delta_time = time_now - time_begin
    # Инициализируем переменную для текущей высоты
    current_height = client.getHeightRange()
    # Выводим текущую высоту
    print(f"Started Height is: {current_height}")

    # Инициализация PID-контроллеров для крена и тангажа
    pid_pitch = PID(0.7, 0.0, 0.1)
    pid_roll = PID(0.7, 0.0, 0.1)
    # Инициализируем переменные ошибок
    pitch_error = 0.0
    roll_error = 0.0
    
    # Устанавливаем время выполнения, по истечению которого дрон совершит посадку 
    while delta_time.total_seconds() < 100:
        try:
            # Обновляем текущее время и рассчитываем время выполнения
            time_now = datetime.now()
            delta_time = time_now - time_begin
            # С помощью сервиса getArucos получаем координаты маркера относительно дрона - это и есть ошибки
            errors = client.getArucos()
            
            # Если камера не видит аруко-маркеров
            if errors == []:  # Потерян маркер
                print('I dont see Aruco')
                # Вызываем функцию регулировки
                ArucoRegulation(pitch_error, roll_error, pid_pitch, pid_roll)
                # Продолжаем выполнения цикла
                continue
            else:
                # Если камера видит аруко-маркеры, записываем в переменные ошибки
                roll_error = errors[0]['pose']['position']['x']
                pitch_error = errors[0]['pose']['position']['y']
                # Вызываем функцию регулировки
                ArucoRegulation(pitch_error, roll_error, pid_pitch, pid_roll)
                # Если ошибки меньше 30 по x и y - снижаем высоту 
                if abs(pitch_error) < 0.3 and abs(roll_error) < 0.3:
                    current_height *= 0.75
                    print("Снижение")
                    # Посылаем в сервис setHeight новую высоту
                    client.setHeight(current_height)
                    print(f"Current Height is: {current_height}")
                if current_height < 0.4:
                    # Останавливаем дрон и завершаем посадку
                    client.setVelXY(0, 0)
                    print("Landing")
                    client.boarding()
                    break
        except KeyboardInterrupt:
            # Обрабатываем прерывание программы с клавиатуры
            print("KeyboardInterrupt detected, landing the drone...")
            break

# Функция регулировки положения дрона относительно аруко-маркера
def ArucoRegulation(pitch, roll, pid_x: PID, pid_y: PID):
    print(f"Adjusting position: pitch={pitch}, roll={roll}")

    # Обновление контроллеров по осям
    pid_x.update_control(pitch)
    pid_y.update_control(roll)

    PID_PITCH = -pid_x.get_control()
    PID_ROLL = pid_y.get_control()  

    # Ограничение управляющих сигналов
    PID_PITCH = constrain(PID_PITCH, 1.1)
    PID_ROLL = constrain(PID_ROLL, 1.1)

    print(f"PID Control: PITCH={PID_PITCH}, ROLL={PID_ROLL}")

    # Устанавливаем скорости для дрона
    client.setVelXY(PID_PITCH, PID_ROLL)
    return (PID_PITCH, PID_ROLL)

# Функция для ограничения значений
def constrain(value, threshold):
    if value > threshold:
        value = threshold
    if value < -threshold:
        value = -threshold
    return value

# Функция обработки входящих сообщений
def OnMes0(mes):
    global last_time
    current_time = datetime.now()
    if last_time is not None:
        delta_time = (current_time - last_time).total_seconds()
    else:
        delta_time = None
    
    last_time = current_time
    
    if delta_time is not None:
        log_message = f"Timestamp: {current_time}, Message: {mes}, Delta Time: {delta_time:.2f} seconds\n"
    else:
        log_message = f"Timestamp: {current_time}, Message: {mes}, Delta Time: N/A\n"

    # Запись сообщения в лог
    with open("messages_log.txt", "a") as file:
        file.write(log_message)

# Создаем объект клиента для взаимодействия с дроном
client = Drone()

# Добавляем обработчик сообщений
client.AddOnMessangeUtils(OnMes0)

# Подключаемся к дрону и выполняем команды
print("connected?", client.connect(ip, port), "\n")
print("VelCorrect", client.setVelXY(0, 0), "\n")
print("takeoff?", client.takeoff(), "\n")
print("Pose aruco: ", client.getCameraPoseAruco(), "\n")
time.sleep(8)
client.setHeight(1.5)
time.sleep(5)         
print("getHeightRange?", client.getHeightRange(), "\n")
print("getOdomOpticflow?", client.getOdomOpticflow(), "\n")
print("Aruco Boarding: ", Aruco_boarding(), "\n")
print("boarding?", client.boarding(), "\n")
print("disconnected?", client.disconnect(), "\n")
