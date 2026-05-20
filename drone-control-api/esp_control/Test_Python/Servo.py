from drone_control_api import Drone
import time


ip = "10.42.0.1"
port = "1233"


client = Drone()


print("connected?", client.connect(ip, port), "\n")


print("setServoAngle?", client.setServoAngle(0.0), "\n") # Поворот на 0 градусов
time.sleep(2.0)
print("setServoAngle?", client.setServoAngle(1.57), "\n") # Поворот на 90 градусов через 2 секунды
time.sleep(2.0)
print("setServoAngle?", client.setServoAngle(3.14), "\n") # Поворот на 180 градусов через 2 секунды
time.sleep(2.0)


client.disconnect()
