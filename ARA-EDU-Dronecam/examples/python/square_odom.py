from drone_control_api import Drone
import time

ip = "192.168.1.91"
client = Drone()

print(client.connect(ip, reset_state=True))
client.takeoff()
time.sleep(5)
client.go_to_xy_drone(1.0, 0.0)
print("Прилетели в первую точку")
client.go_to_xy_drone(1.0, 1.0)
print("Прилетели во вторую точку")
client.go_to_xy_drone(0.0, 1.0)
print("Прилетели в третью точку")
client.go_to_xy_drone(0.0, 1.0)
print("Вернулись на старт")
client.landing()
print("Посадка")