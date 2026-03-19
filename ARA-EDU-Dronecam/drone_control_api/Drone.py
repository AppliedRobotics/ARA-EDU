import asyncio, json, threading, base64, cv2, numpy as np, websockets
from typing import Any, Callable, Dict, Optional
from websockets.protocol import State

_CB = Callable[[dict[str, Any]], Any]

# -------------------------------------------------------------
class Drone:
    """Клиент WebSocket API, скрывающий всю асинхронность."""
    #region Init
    def __init__(self):
        # 1) запускаем фоновый event‑loop
        self.__loop = asyncio.new_event_loop()
        self.__loop_thread = threading.Thread(
            target=self.__loop.run_forever, name="Drone-loop", daemon=True
        )
        self.__loop_thread.start()

        # 2) отложенная инициализация в самом loop’е
        self.__sync(self.__init_async())

    # ---------------- asynchronous part ----------------------
    async def __init_async(self):
        """Создаётся внутри event‑loop’а, поэтому Queue() валиден."""
        # internal state
        self.__ws_control: Optional[websockets.WebSocketClientProtocol] = None
        self.__ws_image:   Optional[websockets.WebSocketClientProtocol] = None

        # ack‑очередь и таблицы ожиданий
        self.__ack_queue: asyncio.Queue[str] = asyncio.Queue()

        self.__pending_fut: Dict[str, asyncio.Future] = {}
        self.__pending_cb: Dict[str, _CB] = {}

        self.__pending_srv: Dict[str, list[asyncio.Future]] = {}

        self.__active_goal: Dict[str, str] = {}

        # place‑holders for receiver task / threads
        self.__recv_task: Optional[asyncio.Task] = None
    #region API methods



    # ++++++++++++++++++++++++++++++ API methods ++++++++++++++++++++++++++++++



    # ============================== Connection ==============================

    
    #region Connection
    def connect(self, ip: str, *, reset_state: bool = False):
        ok = self.__sync(self.__connect(ip))
        if ok and reset_state:
            res = self.__reset_state()
            print(f"Reset state: {res}")
        return ok

    def disconnect(self):
        return self.__sync(self.__disconnect())
    #endregion


    # ============================== Action methods ==============================


    # ------------------------------ Takeoff ------------------------------

    #region Action methods
    def takeoff(self):
        return self.__call_action_sync("takeoff")

    def takeoff_nb(self, callback: Optional[_CB] = None):
        self.__call_action_async("takeoff", callback=callback)

    def takeoff_feedback(self):
        return self.__feedback("takeoff")

    def takeoff_cancel(self):
        return self.__cancel("takeoff")

    # ------------------------------ Landing ------------------------------

    def landing(self):
        return self.__call_action_sync("landing")

    def landing_nb(self, callback: Optional[_CB] = None):
        self.__call_action_async("landing", callback=callback)

    def landing_feedback(self):
        return self.__feedback("landing")

    def landing_cancel(self):
        return self.__cancel("landing")
    
    # ------------------------------ Set height ------------------------------

    def set_height(self, height: float):
        return self.__call_action_sync("set_height", height)

    def set_height_nb(self, height: float, callback: Optional[_CB] = None):
        return self.__call_action_async("set_height", height, callback)

    def set_height_feedback(self):
        return self.__feedback("set_height")

    def set_height_cancel(self):
        return self.__cancel("set_height")

    # ------------------------------ Set yaw ------------------------------

    def set_yaw(self, yaw: float):
        return self.__call_action_sync("set_yaw", yaw)

    def set_yaw_nb(self, yaw: float, callback: Optional[_CB] = None):
        self.__call_action_async("set_yaw", yaw, callback)

    def set_yaw_feedback(self):
        return self.__feedback("set_yaw")

    def set_yaw_cancel(self):
        return self.__cancel("set_yaw")
    
    # ------------------------------ Set yaw relative ------------------------------

    def set_yaw_relative(self, yaw: float):
        return self.__call_action_sync("set_yaw_relative", yaw)

    def set_yaw_relative_nb(self, yaw: float, callback: Optional[_CB] = None):
        self.__call_action_async("set_yaw_relative", yaw, callback)

    def set_yaw_relative_feedback(self):
        return self.__feedback("set_yaw_relative")

    def set_yaw_relative_cancel(self):
        return self.__cancel("set_yaw_relative")
    
    # ------------------------------ Go to xy drone ------------------------------

    def go_to_xy_drone(self, x: float, y: float):
        return self.__call_action_sync("go_to_xy_drone", [x, y])

    def go_to_xy_drone_nb(self, x: float, y: float, callback: Optional[_CB] = None):
        self.__call_action_async("go_to_xy_drone", [x, y], callback)

    def go_to_xy_drone_feedback(self):
        return self.__feedback("go_to_xy_drone")

    def go_to_xy_drone_cancel(self):
        return self.__cancel("go_to_xy_drone")
    
    # ------------------------------ Go to xy odom ------------------------------

    def go_to_xy_odom(self, x: float, y: float):
        return self.__call_action_sync("go_to_xy_odom", [x, y])

    def go_to_xy_odom_nb(self, x: float, y: float, callback: Optional[_CB] = None):
        self.__call_action_async("go_to_xy_odom", [x, y], callback)

    def go_to_xy_odom_feedback(self):
        return self.__feedback("go_to_xy_odom")

    def go_to_xy_odom_cancel(self):
        return self.__cancel("go_to_xy_odom")
    
    # ------------------------------ Go to xy nav ------------------------------

    def go_to_xy_nav(self, x: float, y: float):
        return self.__call_action_sync("go_to_xy_nav", [x, y])

    def go_to_xy_nav_nb(self, x: float, y: float, callback: Optional[_CB] = None):
        self.__call_action_async("go_to_xy_nav", [x, y], callback)

    def go_to_xy_nav_feedback(self):
        return self.__feedback("go_to_xy_nav")

    def go_to_xy_nav_cancel(self):
        return self.__cancel("go_to_xy_nav")
    #endregion


    # ============================== Service methods ==============================


    # ------------------------------ Setters ------------------------------

    #region Service methods
    def set_vel_xy(self, x: float = None, y: float = None):
        try:
            x = float(x)
            y = float(y)
        except:
            raise Exception("x;y can't be None or not float")
        return self.__send_wait_service("set_vel_xy", [x, y])

    def set_vel_xy_yaw(self, x: float = None, y: float = None, yaw: float = None):
        try:
            x = float(x)
            y = float(y)
            yaw = float(yaw)
        except:
            raise Exception("x;y;yaw can't be None or not float")
        return self.__send_wait_service("set_vel_xy_yaw", [x, y, yaw])

    def set_magnet(self, magnet_value: bool = None):
        try:
            magnet_value = bool(magnet_value)
        except:
            raise Exception("magnet_value can't be None or not bool")
        return self.__send_wait_service("set_magnet", magnet_value)
    
    def set_diod(self, r: float = None, g: float = None, b: float = None):
        try:
            r = float(r)
            g = float(g)
            b = float(b)
        except:
            raise Exception("r;g;b can't be None or not float")
        return self.__send_wait_service("set_diod", [r, g, b])

    def set_beeper(self, power: float = None, freq: float = None):
        try:
            power = float(power)
            freq = float(freq)
        except:
            raise Exception("power;freq can't be None or not float")
        return self.__send_wait_service("set_beeper", [power, freq])
    
    def set_zero_odom_opticflow(self):
        return self.__send_wait_service("set_zero_odom_opticflow")

    
    # ------------------------------ Getters ------------------------------
    
    def get_odom_opticflow(self):
        return self.__send_wait_service("get_odom_opticflow")
    def get_lidar(self):
        return self.__send_wait_service("get_lidar")
    def get_rpy(self):
        return self.__send_wait_service("get_rpy")
    def get_height_barometer(self):
        return self.__send_wait_service("get_height_barometer")
    def get_height_rangefinder(self):
        return self.__send_wait_service("get_height_rangefinder")
    def get_arm(self):
        return self.__send_wait_service("get_arm")
    def get_arucos(self):
        return self.__send_wait_service("get_arucos")
    def get_camera_pose_aruco(self):
        return self.__send_wait_service("get_camera_pose_aruco")
    def get_light(self):
        return self.__send_wait_service("get_light")
    def get_ultrasonic(self):
        return self.__send_wait_service("get_ultrasonic")
    def get_blobs(self):
        return self.__send_wait_service("get_blobs")
    def get_detections(self):
        return self.__send_wait_service("get_detections")
    def get_nav_pose(self):
        return self.__send_wait_service("get_nav_pose")

    def get_image(self):
        return self.__sync(self.__get_image_coro())

    #endregion
    #endregion
    #region Utils



    # ++++++++++++++++++++++++++++++ Utils ++++++++++++++++++++++++++++++



    # ============================== Connection utils ==============================

    
    #region Connection utils
    def __sync(self, awaitable):
        """
        • coroutine → запускаем в фоновом loop’е, ждём результат;
        • asyncio.Future → тоже ждём в фоновом loop’е, чтобы он успел
        выполниться, а не выстрелить InvalidStateError.
        """
        if asyncio.iscoroutine(awaitable):
            fut = asyncio.run_coroutine_threadsafe(awaitable, self.__loop)
            return fut.result()

        if isinstance(awaitable, asyncio.Future):
            # оборачиваем Future в корутину «await fut» и
            # дожидаемся её тем же run_coroutine_threadsafe
            async def _wait(f):         # небольшая обёртка
                return await f
            fut = asyncio.run_coroutine_threadsafe(_wait(awaitable), self.__loop)
            return fut.result()

        raise TypeError("expected coroutine or Future")
    
    async def __connect(self, ip: str):
        try:
            self.__ws_control = await websockets.connect(f"ws://{ip}:1233/ws/api/control")
            self.__ws_image = await websockets.connect(f"ws://{ip}:1235/ws/api/image")

            self.__recv_task = asyncio.create_task(self.__control_receiver(), name="control-recv")
            
            return True
        
        except ConnectionRefusedError as err:
            print(err.strerror)
            return False
        
    async def __disconnect(self):
        if self.__ws_control and self.__ws_control.state == State.OPEN:
            await self.__ws_control.close()
        if self.__recv_task:
            self.__recv_task.cancel()

        closed = self.__ws_control is None or self.__ws_control.state == State.CLOSED
        return closed


    # ============================== Action methods utils ==============================


    #region Action methods utils
    def __call_action_sync(self, method: str, args: Any | None = None):
        return self.__sync(self.__call_action_sync_coro(method, args))

    async def __call_action_sync_coro(self, method: str, args):
        req_id = await self.__start_goal(method, args)
        fut = self.__pending_fut[req_id] = self.__loop.create_future()
        return await fut
    

    def __call_action_async(
        self,
        method: str,
        args: Any | None = None,
        callback: Optional[_CB] = None,
    ) -> dict[str, str]:
        """Неблокирующий вызов. Сразу отдаёт {'id':.., 'status':'goal_sent'}."""
        req_id = self.__sync(
            self.__call_action_async_coro(method, args, callback)
        )
        return {"id": req_id, "status": "goal_sent"}

    async def __call_action_async_coro(self, method, args, callback):
        req_id = await self.__start_goal(method, args)
        if callback:
            # сохраняем, но НЕ удаляем — сделает _receiver после финала
            self.__pending_cb[req_id] = callback
        return req_id            # ← важно, чтобы верхний метод получил id
    

    def __feedback(self, method: str):
        return self.__sync(self.__feedback_coro(method))

    async def __feedback_coro(self, method: str):
        req_id = self.__active_goal.get(method)
        if not req_id:
            return {"error": f"{method} is not active"}
        fut = self.__pending_fut[req_id] = self.__loop.create_future()
        await self.__ws_control.send(json.dumps({"method": method, "command": "feedback", "id": req_id}))  # type: ignore[arg-type]
        return await fut
    

    def __cancel(self, method: str):
        return self.__sync(self.__cancel_coro(method))

    async def __cancel_coro(self, method: str):
        req_id = self.__active_goal.get(method)
        if not req_id:
            return {"error": f"{method} is not active"}
        fut = self.__pending_fut[req_id] = self.__loop.create_future()
        await self.__ws_control.send(json.dumps({"method": method, "command": "cancel", "id": req_id}))  # type: ignore[arg-type]
        return await fut
    

    async def __start_goal(self, method: str, args):
        payload = {"method": method, "command": "start"}
        if args is not None:
            payload["args"] = args
        await self.__ws_control.send(json.dumps(payload))  # type: ignore[arg-type]
        req_id = await self.__ack_queue.get()
        self.__active_goal[method] = req_id
        return req_id
    

    async def __control_receiver(self):
        try:
            async for raw in self.__ws_control:
                data = json.loads(raw)
                # print("[RX]", data)          # включите для отладки

                # 1) ACK от action
                if data.get("ack"):
                    await self.__ack_queue.put(data["id"])
                    continue

                # 2) ответ обычного сервиса
                if "method" in data and (
                    "response" in data or "error" in data or "result" in data
                ):
                    method_name = data["method"]
                    queue = self.__pending_srv.get(method_name)
                    if queue:
                        fut = queue.pop(0)                   # FIFO
                        if not queue:                        # очередь опустела
                            self.__pending_srv.pop(method_name, None)
                        if not fut.done():
                            fut.set_result(data)
                    continue

                # 3) ответ action-goal
                req_id = data.get("id")
                if req_id:
                    fut = self.__pending_fut.pop(req_id, None)
                    if fut and not fut.done():
                        fut.set_result(data)

                    cb = self.__pending_cb.get(req_id)
                    if cb and "result" in data:
                        try:
                            cb(data)
                        finally:
                            self.__pending_cb.pop(req_id, None)
                    continue
        except asyncio.CancelledError:
            pass
    #endregion


    # ============================== Service methods utils ==============================


    #region Service methods utils
    def __send_wait_service(self, method: str, params=None):
        """
        Отправить сервис-запрос и блокирующе дождаться ответа.
        • Future ставится в очередь ДО отправки, чтобы не было гонки.
        """
        # 1) подготовить Future и очередь
        fut = self.__loop.create_future()
        self.__pending_srv.setdefault(method, []).append(fut)   # ← гарантируем список



        # 2) сформировать JSON-запрос (без id – его создаст сервер)
        payload = {"method": method}
        if params is not None:
            payload["params"] = params

        # 3) отправить запрос
        self.__sync(self.__ws_control.send(json.dumps(payload)))

        # 4) дождаться результата
        reply: Dict = self.__sync(fut)

        # 5) вернуть «чистый» ответ, как раньше
        if "response" in reply:
            return reply["response"]
        if "error" in reply:
            return f"error: {reply['error']}"
        return reply.get("result", "unknown")
    
    
    async def __get_image_coro(self):
        if not self.__ws_image or self.__ws_image.closed:
            return None
        
        await self.__ws_image.send(json.dumps({"method": "get_image"}))

        raw = await self.__ws_image.recv()

        data: Dict = json.loads(raw)
        b64 = data.get("image")
        if not b64:
            return None

        img_bytes = base64.b64decode(b64)
        arr = np.frombuffer(img_bytes, np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)

        return img
    #endregion
    

    # ============================== Legacy utils ==============================


    #region Legacy utils
    def send_mess(self, message: str):
        self.__sync(self.__ws_control.send(message))

    def recv_mess_control(self):
        if self.__ws_control.state is not State.OPEN:
            return json.dumps({"error": "no connection"})
        return self.__sync(self.__ws_control.recv())
    #endregion


    # ============================== Private methods ==============================

    #region Private methods
    def __reset_state(self):
        return self.__send_wait_service("_reset_state")
    #endregion
    #endregion
