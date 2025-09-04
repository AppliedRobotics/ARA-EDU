import time
from drone_control_api import Drone
from drone_control_api.pid import PID
import datetime
import math

ip = "10.42.0.1"
port = "1233"

last_time = None


def BlobRegulation():
   # Настройка PID регулятора для компоненты ROLL(в данном случае P-регулятор)
   pid_roll = PID(0.01, 0.0, 0.0)

   # Центральный пиксель изображения, приходящего с камеры
   camera_center_x = 320/2
   camera_center_y = 240/2

   # Целевая площадь блоба, при которой дрон приступит к облёту препятствия
   target_area = 2000

   # Минимальная площадь блоба
   threshold_area = 800

   # Инициализация площади блоба
   blob_area = 0

   while True:
       # Для каждого блоба из обнаруженных
       for blob in client.getBlobs():
           # Считаем приблизительную площадь блоба
           blob_area = blob["size"]["x"] * blob["size"]["y"]
           # Если площадь больше пороговой
           if blob_area > threshold_area:
               print(f"Обнаружен нужный блоб с центром: {blob['center']}, площадью: {blob_area}")
               # Считаем ошибку по ROLL компоненте
               roll_error = blob["center"]["x"] - camera_center_x
               # Выравниваемся и подлетаем с малой скоростью
               Regulation(roll_error,  pid_roll=pid_roll)
            
           else:
               # Сбрасываем скорость
               client.setVelXY(0, 0)
               print("Нет подходящих блобов. Обнуляю скорость")

       # Если площадь больше целевой
       if blob_area > target_area:
               print("Облёт препятствия")
               # Вызов функции облёта
               avoid()
               break
       
def avoid():
   # Обнуляем одометрию
   client.setZeroOdomOpticflow()
   # Летим вправо на 0.5 метра
   client.gotoXYdrone(0.0, -0.5)
   # Обнуляем одометрию
   client.setZeroOdomOpticflow()
   # Летим на 1 метр вперед
   client.gotoXYdrone(1.0, 0.0)
   # Обнуляем одометрию
   client.setZeroOdomOpticflow()
   # Летим влево на 0.5 метра
   client.gotoXYdrone(0.0, 0.5)
  

def Regulation(roll_error, pid_roll:PID):
   # Инициализация постоянной скорости PITCH
   PITCH_vel = 0.0
   # Если ошибка ROLL небольшая - летим вперёд с малой скоростью
   if roll_error < 15:
       PITCH_vel = 0.6
   # Регулировка скорости ROLL
   pid_roll.update_control(roll_error)
   PID_ROLL = pid_roll.get_control()
   # Ограничиваем скорость пороговым значением
   PID_ROLL = constrain(PID_ROLL, 1.0)
   print(f"Корректировка PID_ROLL : {PID_ROLL}")
   # Устанавливаем скорость
   client.setVelXY(PITCH_vel, PID_ROLL)

#Функция для ограничения значения переменной
def constrain(value, threshold):
   if value > threshold:
       value = threshold
   if value < -threshold:
       value = -threshold
   return value       

client = Drone()

print("connected?", client.connect(ip, port), "\n")
print("VelCorrect", client.setVelXYYaw(0,0,0),"\n")
print("takeoff?", client.takeoff(), "\n")

print("Blobs: ", client.getBlobs(), "\n")
print(BlobRegulation())

print("VelCorrect", client.setVelXYYaw(0,0,0),"\n")
print("boarding?", client.boarding(), "\n")
