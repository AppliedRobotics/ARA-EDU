from drone_control_api import Drone
import time


ip = "192.168.1.92"
client = Drone()

print(client.connect(ip, reset_state=True))
client.takeoff()
time.sleep(5)
search = True
client.set_vel_xy_yaw(0.0, 0.0, 0.5)
while search:
    detection = client.get_detections()
    if detection:
        if detection['boxes'][0]["name"] == "human":
            client.set_vel_xy_yaw(0.0, 0.0, 0.0)
            search = False
            print("Человек найден")
print(client.landing())
