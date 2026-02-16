import asyncio
import websockets
import json
from websockets.protocol import State
from threading import Thread, Event
import base64
import cv2
import numpy as np

class Drone:
    #region main methods
    def __init__(self):
        self.__websocket_control = websockets.WebSocketClientProtocol()
        self.__loop = asyncio.get_event_loop()

        self.__websocket_image = websockets.WebSocketClientProtocol()
        self.__websocket_utils = websockets.WebSocketClientProtocol()
        
        self.__thread_utils = None
        self.__thread_image = None
        self.__stop_event_threads = Event()
        self.OnMessangeUtils = []
        self.OnMessangeImage = []

    def connect(self, ip: str, port: str):
        if not isinstance(ip, str) or not isinstance(port, str):
            raise Exception("ip and port could be string")
        result = self.__loop.run_until_complete(self._connect(ip, port))
        
        # self.startThread(self.__thread_utils, self._message_handler_utils, f"ws://{ip}:{port}/ws/api/util")
        # self.startThread(self.__thread_image, self._message_handler_image, f"ws://{ip}:{port}/ws/api/image")
        
        return result
    
    async def _connect(self, ip: str, port: str):
        try:
            self.__websocket_control = await websockets.connect(f"ws://{ip}:{port}/ws/api/control")
            self.__websocket_utils = await websockets.connect(f"ws://{ip}:{port}/ws/api/util")
            self.__websocket_image = await websockets.connect(f"ws://{ip}:{1235}/ws/api/image")
            return True
        except Exception as e:
            print("Error connect to drone", e)
            return False

    def disconnect(self):
        result = self.__loop.run_until_complete(self._disconnect())
        return result
    async def _disconnect(self):
        if self.__websocket_control and self.__websocket_control.state == State.OPEN:
            await self.__websocket_control.close()
        
        self.__stop_event_threads.set()
        if self.__thread_utils:
            self.__thread_utils.join()
        if self.__thread_image:
            self.__thread_image.join()

        if self.__websocket_control.state == State.CLOSED:
            return True
        else:
            return False

    def send_mess(self, message):
        self.__loop.run_until_complete(self._send_mess(message))
    async def _send_mess(self, message):
        if self.__websocket_control.state is State.OPEN:
            await self.__websocket_control.send(message)
    
    def recv_mess_control(self):
        if self.__websocket_control.state is not State.OPEN:
            return json.dumps({"error": "no connection"})
        mess = self.__loop.run_until_complete(self._recv_mess_control())
        return mess
    async def _recv_mess_control(self):
        if self.__websocket_control.state is State.OPEN:
            # await asyncio.sleep(0)
            try:
                if self.__websocket_control:
                    message = await self.__websocket_control.recv()
                    return message
                
            except websockets.exceptions.ConnectionClosed:
                await self._disconnect()
                return "WebSocket connection closed"
    


    
    def recv_mess_image(self):
        if self.__websocket_image.state is not State.OPEN:
            return json.dumps({"error": "no connection"})
        mess = self.__loop.run_until_complete(self._recv_mess_image())
        return mess
    async def _recv_mess_image(self):
        if self.__websocket_image.state is State.OPEN:
            try:
                if self.__websocket_image:
                    message = await self.__websocket_image.recv()
                    return message
                
            except websockets.exceptions.ConnectionClosed:
                await self._disconnect()
                return "WebSocket connection closed"
    
    def recv_mess_utils(self):
        if self.__websocket_utils.state is not State.OPEN:
            return json.dumps({"error": "no connection"})
        mess = self.__loop.run_until_complete(self._recv_mess_utils())
        return mess
    async def _recv_mess_utils(self):
        if self.__websocket_utils.state is State.OPEN:
            try:
                if self.__websocket_utils:
                    message = await self.__websocket_utils.recv()
                    return message
                
            except websockets.exceptions.ConnectionClosed:
                await self._disconnect()
                return "WebSocket connection closed"
    


    def AddOnMessangeUtils(self, handler):
        self.OnMessangeUtils.append(handler)
    def AddOnMessangeImage(self, handler):
        self.OnMessangeImage.append(handler)
    # waiting for remove OnMessangeUtils interfaces

    #region Threads
    def start_websocket_client_in_thread(self, uri, _message_handler_endless):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        try:
            loop.run_until_complete(_message_handler_endless(uri))
        except asyncio.CancelledError:
            print("WebSocket client was cancelled.")
        finally:
            loop.close()

    def startThread(self, thread, _message_handler_endless, uri):
        self.__stop_event_threads = Event()
        thread = Thread(target=self.start_websocket_client_in_thread, args=(uri, _message_handler_endless,))
        thread.daemon = True
        thread.start()
    
    async def _message_handler_utils(self, uri):
        async with websockets.connect(uri) as websocket:
            while not self.__stop_event_threads.is_set():
                try:
                    message = await websocket.recv()
                    if message:
                        message_data = json.loads(message)
                        for item in self.OnMessangeUtils:
                            item(message_data)
                except json.JSONDecodeError:
                    pass
                except:
                    await self._disconnect()
    
    async def _message_handler_image(self, uri):
        # try:
        async with websockets.connect(uri) as websocket:
            while not self.__stop_event_threads.is_set():
                # try:
                message = await websocket.recv()
                if message:
                    message_data = json.loads(message)
                    for item in self.OnMessangeImage:

                        image_data = base64.b64decode(message_data['image'])
                        np_array = np.frombuffer(image_data, np.uint8)
                        img = cv2.imdecode(np_array, cv2.IMREAD_COLOR)

                        item(img)
                    # except json.JSONDecodeError:
                    #     pass
                    # except:
                    #     await self._disconnect()
        # except:
            # raise Exception("Cant connect to image WS")

    #endregion

    #endregion

    def __parse_get(self, json_string: str):
        try:
            json_data = json.loads(json_string)
            data = json_data["response"]
            return data
        except (json.JSONDecodeError, KeyError):
            try:
                error_data = json.loads(json_string)
                error = error_data["error"]
                return f"error: {error}"
            except:
                return "unknown error"
    def __send_wait_method_temp(self, method: str, params = None):
        if params == None:
            data = {"method":method}
        else:
            data = {"method":method, "params" : params}
        
        json_string = json.dumps(data)
        self.send_mess(json_string)
        mess = self.recv_mess_control()

        res = self.__parse_get(mess)
        return res
    
    
    def takeoff(self):
        return self.__send_wait_method_temp("takeoff")
    def boarding(self):
        return self.__send_wait_method_temp("boarding")
    def setZeroOdomOpticflow(self):
        return self.__send_wait_method_temp("setZeroOdomOpticflow")

    
    def getOdomOpticflow(self):
        return self.__send_wait_method_temp("getOdomOpticflow")
    def getLidar(self):
        return self.__send_wait_method_temp("getLidar")
    def getRPY(self):
        return self.__send_wait_method_temp("getRPY")
    def getHeightBarometer(self):
        return self.__send_wait_method_temp("getHeightBarometer")
    def getHeightRange(self):
        return self.__send_wait_method_temp("getHeightRange")
    def getArm(self):
        return self.__send_wait_method_temp("getArm")
    def getArucos(self):
        return self.__send_wait_method_temp("getArucos")
    def getCameraPoseAruco(self):
        return self.__send_wait_method_temp("getCameraPoseAruco")
    def getLight(self):
        return self.__send_wait_method_temp("getLight")
    def getUltrasonic(self):
        return self.__send_wait_method_temp("getUltrasonic")
    def getBlobs(self):
        return self.__send_wait_method_temp("getBlobs")
    
    def getImage(self):
        message = self.recv_mess_image()
        message_data = json.loads(message)
        image_data = base64.b64decode(message_data["image"])
        np_array = np.frombuffer(image_data, np.uint8)
        img = cv2.imdecode(np_array, cv2.IMREAD_COLOR)
        return img
    
    def getUtilsData(self):
        message = self.recv_mess_utils()
        message_data = json.loads(message)
        return message_data["response"]
    # message_data["response"]


    def setYaw(self, yaw: float = None):
        try:
            yaw = float(yaw)
        except:
            raise Exception("yaw cant be None or not float")
        return self.__send_wait_method_temp("setYaw", yaw)
    
    def setVelXY(self, x: float = None, y: float = None):
        try:
            x = float(x)
            y = float(y)
        except:
            raise Exception("x;y cant be None or not float")
        return self.__send_wait_method_temp("setVelXY", [x, y])
    
    def setVelXYYaw(self, x: float = None, y: float = None, yaw: float = None):
        try:
            x = float(x)
            y = float(y)
            yaw = float(yaw)
        except:
            raise Exception("x;y;yaw cant be None or not float")
        return self.__send_wait_method_temp("setVelXYYaw", [x, y, yaw])

    def gotoXYdrone(self, x: float = None, y: float = None):
        try:
            x = float(x)
            y = float(y)
        except:
            raise Exception("x;y cant be None or not float")
        return self.__send_wait_method_temp("gotoXYdrone", [x, y])
    
    def gotoXYodom(self, x: float = None, y: float = None):
        try:
            x = float(x)
            y = float(y)
        except:
            raise Exception("x;y cant be None or not float")
        return self.__send_wait_method_temp("gotoXYodom", [x, y])
    
    def setHeight(self, height: float = None):
        try:
            height = float(height)
        except:
            raise Exception("height cant be None or not float")
        return self.__send_wait_method_temp("setHeight", height)
    
    def setDiod(self, r: float = None, g: float = None, b: float = None):
        try:
            r = float(r)
            g = float(g)
            b = float(b)
        except:
            raise Exception("r;g;b cant be None or not float")
        return self.__send_wait_method_temp("setDiod", [r, g, b])

    def setBeeper(self, power: float = None, freq: float = None):
        try:
            power = float(power)
            freq = float(freq)
        except:
            raise Exception("power;freq cant be None or not float")
        return self.__send_wait_method_temp("setBeeper", [power, freq])    
    
    def setShoot(self, time_shoot: float = None):
        try:
            time_shoot = float(time_shoot)
        except:
            raise Exception("time_shoot cant be None or not float")
        return self.__send_wait_method_temp("setShoot", time_shoot)
    
    def setServoAngle(self, angle: float = None):
        try:
            angle = float(angle)
        except:
            raise Exception("angle cant be None or not float")
        return self.__send_wait_method_temp("setServoAngle", angle)  
    
    def setMagnet(self, val: bool = None):
        try:
            val = bool(val)
        except:
            raise Exception("val cant be None or not bool")
        return self.__send_wait_method_temp("setMagnet", val)
    
        
    