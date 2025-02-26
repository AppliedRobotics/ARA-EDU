from drone_control_api import Drone   
import datetime
import cv2
import time

# ip = "127.0.0.1"
ip = "trackingcam3.local"
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

    # with open("messages_log.txt", "a") as file:
    #     file.write(log_message)
    # print(log_message)
    
def OnMesImage(img):
    print(f"img = {img}\n")

    cv2.imwrite('img', img)
    # cv2.imshow('img', img)


client = Drone()

# client.AddOnMessangeUtils(OnMes0)
# # client.AddOnMessangeImage(OnMesImage)


print("connected?", client.connect(ip, port), "\n")

print("takeoff?", client.takeoff(), "\n")
print("boarding?", client.boarding(), "\n")
print("setZeroOdomOpticflow?", client.setZeroOdomOpticflow(), "\n")

print("getOdomOpticflow?", client.getOdomOpticflow(), "\n")
print("getLidar?", client.getLidar(), "\n")
print("getRPY?", client.getRPY(), "\n")
print("getHeightBarometer?", client.getHeightBarometer(), "\n")
print("getHeightRange?", client.getHeightRange(), "\n")
print("getArm?", client.getArm(), "\n")

print("setYaw?", client.setYaw(0), "\n")
print("setVelXY?", client.setVelXY(0, 0), "\n")
print("setVelXYYaw?", client.setVelXYYaw(0, 0, 0), "\n")
print("gotoXYdrone?", client.gotoXYdrone(0, 0), "\n")
print("gotoXYodom?", client.gotoXYodom(0, 0), "\n")
print("setHeight?", client.setHeight(0), "\n")
print("getArucos?", client.getArucos(), "\n")
print("getCameraPoseAruco?", client.getCameraPoseAruco(), "\n")
print("setMagnet?", client.setMagnet(False), "\n")
print("getBlobs?", client.getBlobs(False), "\n")


print(f"getImage = {client.getImage()}")
imgg = client.getImage()
cv2.imshow("imgg", imgg)
cv2.waitKey(0)


utils = client.getUtilsData()
print(utils)


print("disconnected?", client.disconnect(), "\n")