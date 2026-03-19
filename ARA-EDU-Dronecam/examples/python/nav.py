from drone_control_api import Drone
import time


ip = "192.168.1.91"
client = Drone()

print(client.connect(ip, reset_state=True))
client.takeoff()
client.go_to_xy_nav(3.0, 2.0)
client.landing()

