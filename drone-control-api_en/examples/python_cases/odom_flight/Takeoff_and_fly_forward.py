from drone_control_api import Drone
import time

ip = "192.168.1.197"
port = "1233"

client = Drone()



print("connected?", client.connect(ip, port), "\n")

print("takeoff?", client.takeoff(), "\n")
time.sleep(10)
print("setZeroOdomOpticflow?", client.setZeroOdomOpticflow(), "\n")
print("gotoXYdrone?", client.gotoXYdrone(1.5, 0), "\n") #полет вперед по одометрии на 1.5м
#print("gotoXYodom?", client.gotoXYodom(1.5, 0), "\n") #полет вперед относительно текущей позиции дрона
print("boarding?", client.boarding(), "\n")

print("disconnected?", client.disconnect(), "\n")