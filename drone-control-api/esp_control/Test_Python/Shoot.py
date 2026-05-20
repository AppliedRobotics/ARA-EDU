from drone_control_api import Drone
import time


ip = "10.42.0.1"
port = "1233"


client = Drone()


print("connected?", client.connect(ip, port), "\n")


client.setShoot(0.15) # Одиночный выстрел
time.sleep(5.0)
client.setShoot(0.15) # Одиночный выстрел через 5 секунд
time.sleep(5.0)
client.setShoot(0.15) # Одиночный выстрел через 5 секунд
time.sleep(2.0)

# # 10 одиночных выстрелов через каждые 2 секунды
# for _ in range(10):
#     client.setShoot(0.15)
#     time.sleep(2.0)


client.disconnect()
