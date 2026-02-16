from drone_control_api import Drone
from drone_control_api.pid import PID  
import datetime
import cv2

ip = "10.42.0.1"
port = "1233"

last_time = None
def OnMes0(mes):
    global last_time
    current_time = datetime.datetime.now()
    if last_time is not None:
        delta_time = (current_time - last_time).total_seconds()
    else:
        delta_time = None
   
    last_time = current_time
   
    if delta_time is not None:
        log_message = f"Timestamp: {current_time}, Message: {mes}, Delta Time: {delta_time:.2f} seconds\n"
    else:
        log_message = f"Timestamp: {current_time}, Message: {mes}, Delta Time: N/A\n"
   
def OnMesImage(img):
    print(f"img = {img}\n")
    cv2.imwrite('img', img)
    
client = Drone()

client.AddOnMessangeUtils(OnMes0)
client.AddOnMessangeImage(OnMesImage)

print("connected?", client.connect(ip, port), "\n")
# print("takeoff?", client.takeoff(), "\n")
# print("boarding?", client.boarding(), "\n")
# print("setZeroOdomOpticflow?", client.setZeroOdomOpticflow(), "\n")
# print("getOdomOpticflow?", client.getOdomOpticflow(), "\n")
# print("getLidar?", client.getLidar(), "\n")
# print("getRPY?", client.getRPY(), "\n")
# print("getHeightBarometer?", client.getHeightBarometer(), "\n")
# print("getHeightRange?", client.getHeightRange(), "\n")
# print("getArm?", client.getArm(), "\n")
# print("setYaw?", client.setYaw(0), "\n")
# print("setVelXY?", client.setVelXY(0, 0), "\n")
# print("setVelXYYaw?", client.setVelXYYaw(0, 0, 0), "\n")
# print("gotoXYdrone?", client.gotoXYdrone(0, 0), "\n")
# print("gotoXYodom?", client.gotoXYodom(0, 0), "\n")
# print("setHeight?", client.setHeight(0), "\n")
# print("getArucos?", client.getArucos(), "\n")
# print("getCameraPoseAruco?", client.getCameraPoseAruco(), "\n")
# print("setMagnet?", client.setMagnet(False), "\n")
# print("getBlobs?", client.getBlobs(), "\n")

# print(f"getImage = {client.getImage()}")
# imgg = client.getImage()
# cv2.imshow("imgg", imgg)
# cv2.waitKey(0)

# utils = client.getUtilsData()
# print(utils)

print("disconnected?", client.disconnect(), "\n")