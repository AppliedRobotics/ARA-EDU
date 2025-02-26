import asyncio
import websockets
import json
from websockets.protocol import State
from threading import Thread

class Drone:
    #region main methods
    def __init__(self):
        self.__websocket_control = websockets.WebSocketClientProtocol()
        self.__websocket_utils = websockets.WebSocketClientProtocol()
        self.__loop = asyncio.get_event_loop()
        
        self.__task_utils = None
        self.OnMessangeUtils = []

    def connect(self, ip: str, port: str):
        if not isinstance(ip, str) or not isinstance(port, str):
            raise Exception("ip and port could be string")
        result = self.__loop.run_until_complete(self._connect(ip, port))
        self.startThreadUtils()
        return result
    async def _connect(self, ip: str, port: str):
        try:
            self.__websocket_control = await websockets.connect(f"ws://{ip}:{port}/ws/api/control")
            self.__websocket_utils = await websockets.connect(f"ws://{ip}:{port}/ws/api/util")
            return True
        except:
            return False

    def disconnect(self):
        result = self.__loop.run_until_complete(self._disconnect())
        return result
    async def _disconnect(self):
        if self.__websocket_utils and self.__websocket_utils.state == State.OPEN:
            await self.__websocket_utils.close()
        if self.__websocket_control and self.__websocket_control.state == State.OPEN:
            await self.__websocket_control.close()

        if self.__task_utils:
            self.__task_utils.cancel()
            try:
                await self.__task_utils
            except asyncio.CancelledError:
                pass

        if self.__websocket_utils.state == State.CLOSED and self.__websocket_control.state == State.CLOSED:
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
            await asyncio.sleep(10)
            try:
                if self.__websocket_control:
                    message = await self.__websocket_control.recv()
                    return message
                
            except websockets.exceptions.ConnectionClosed:
                await self._disconnect()
                return "WebSocket connection closed"
    



    def AddOnMessangeUtils(self, handler):
        self.OnMessangeUtils.append(handler)


    def startThreadUtils(self):
        if not self.__task_utils:
            self.__task_utils = self.__loop.create_task(self._message_handler_utils())
            # self.__task_utils = asyncio.to_thread(self.__loop.create_task(self._message_handler_utils()))
    
    async def _message_handler_utils(self):
        while self.__websocket_utils and self.__websocket_utils.state is State.OPEN:
            try:
                message = await self.__websocket_utils.recv()
                if message:
                    message_data = json.loads(message)
                    for item in self.OnMessangeUtils:
                        item(message_data)
            except json.JSONDecodeError:
                pass
            except:
                await self._disconnect()
            

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
    


