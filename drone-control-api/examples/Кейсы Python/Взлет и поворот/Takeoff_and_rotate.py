from drone_control_api import Drone
import time

ip = "192.168.1.197"
port = "1233"

client = Drone()



print("connected?", client.connect(ip, port), "\n")

print("takeoff?", client.takeoff(), "\n")
time.sleep(10)
print("setYaw?", client.setYaw(1.57), "\n") #поворот по часовой стрелке относительно положения, в котором дрон был включен (угол задается в радианах)
print("boarding?", client.boarding(), "\n")

print("disconnected?", client.disconnect(), "\n")