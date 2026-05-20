from drone_control_api import Drone
import time


ip = "10.42.0.1"
port = "1233"


client = Drone()


print("connected?", client.connect(ip, port), "\n")


print("setMagnet?", client.setMagnet(True), "\n") # Включение магнита 
time.sleep(10.0)
print("setMagnet?", client.setMagnet(False), "\n") # Выключение магнита через 10 секунд
time.sleep(2.0)


client.disconnect()
