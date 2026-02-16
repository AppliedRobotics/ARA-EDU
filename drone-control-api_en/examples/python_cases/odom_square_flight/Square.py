from drone_control_api import Drone
import time

ip = "192.168.1.197"
port = "1233"

client = Drone()



print("connected?", client.connect(ip, port), "\n")

print("takeoff?", client.takeoff(), "\n")
time.sleep(10)

print("setZeroOdomOpticflow?", client.setZeroOdomOpticflow(), "\n")
print("gotoXYodom?", client.gotoXYodom(1, 0), "\n")
print("gotoXYodom?", client.gotoXYodom(1, 1), "\n")
print("gotoXYodom?", client.gotoXYodom(0, 1), "\n")
print("gotoXYodom?", client.gotoXYodom(0, 0), "\n")

# print("gotoXYdrone?", client.gotoXYdrone(1, 0), "\n")
# print("gotoXYdrone?", client.gotoXYdrone(0, 1), "\n")
# print("gotoXYdrone?", client.gotoXYdrone(-1, 0), "\n")
# print("gotoXYdrone?", client.gotoXYdrone(0, -1), "\n")

print("boarding?", client.boarding(), "\n")

print("disconnected?", client.disconnect(), "\n")