# Импорт необходимых библиотек и сообщений ROS 2
import rclpy
from rclpy.impl.rcutils_logger import Once
from rclpy.node import Node
from eagle_eye_msgs.msg import MarkerArray, Altitude # Пользовательские сообщения для маркеров и высоты
from eagle_eye_msgs.srv import TakeOffBoard, SetHeight # Пользовательские сервисы для взлета/посадки и установки высоты
from geometry_msgs.msg import Twist # Сообщение для управления скоростью
import time # Для работы со временем

class ArucoBoarding(Node):
    def __init__(self):
        super().__init__("aruco_boarding")
        self.get_logger().info("Aruco Boarding node initialized")
        # Подписка на топик с маркерами ArUco
        self.marker_sub = self.create_subscription(MarkerArray, "arucos", self.marker_callback, 10)
        # Издатель для отправки команд скорости
        self.cmd_publisher = self.create_publisher(Twist, "cmd_vel", 10)
        # Создаем клиент сервиса для взлета/посадки
        self.takeoff_board_client = self.create_client(TakeOffBoard, "/take_off_board")
        # Создаем клиент сервиса для установки высоты
        self.set_height_client = self.create_client(SetHeight, "/set_height")


        # Инициализируем переменные для ошибок регулирования по осям X (крен) и Y (тангаж)
        self.roll_error = 0.0
        self.pitch_error = 0.0
        
        # Начальная целевая высота для дрона
        self.target_height = 1.5
        self.initial_target_height = self.target_height

        # Инициализация PID-регуляторов для крена и тангажа
        self.pid_roll = PID(0.7, 0.000, 0.1)
        self.pid_pitch = PID(0.7, 0.000, 0.1)

        # Флаг состояния взлета
        self.takeoff = False

        # Флаг, указывающий на процесс снижения
        self.descending_flag = False  
        # Время последнего обнаружения маркера, используется для таймаута
        self.last_marker_seen = 0  

        # Флаг состояния посадки
        self.boarding = False

        # Вызов сервиса для взлета (takeoff=True) и запрета посадки(board=False)
        self.call_takeoff_board_service(takeoff=True, board=False)

        # Инициализация и публикация нулевых скоростей для остановки дрона
        test_msg = Twist()
        test_msg.linear.x = 0.0
        test_msg.linear.y = 0.0
        test_msg.angular.z = 0.0
        self.cmd_publisher.publish(test_msg)
    
    def marker_callback(self, msg):
        # Обработка маркеров только после успешного взлета
        if self.takeoff:
            aruco_markers = msg.markers

            current_time = time.time()
            # Если обнаружены маркеры
            if len(aruco_markers) > 0:
                # Получаем ошибки по осям X (крен) и Y (тангаж) из первого маркера
                self.roll_error = aruco_markers[0].pose.pose.position.x
                self.pitch_error = aruco_markers[0].pose.pose.position.y
                self.get_logger().info(f"Roll error: {self.roll_error}, Pitch error: {self.pitch_error}")
                # Вызов функции регулирования на основе ошибок
                self.ArucoRegulation(self.pitch_error, self.roll_error)
                # Обновляем время последнего обнаружения маркера
                self.last_marker_seen = current_time
                # Проверяем, можно ли начать спуск (если ошибки малы и не идет процесс снижения)
                if (abs(self.pitch_error) < 0.1 and abs(self.roll_error) < 0.1) and not self.descending_flag:  
                    self.get_logger().info("Descending")
                    # Уменьшаем целевую высоту на 50%
                    self.target_height *= 0.5
                    self.get_logger().info(f"Target height: {self.target_height}")
                    # Вызов сервиса для установки новой целевой высоты
                    self.call_set_height_service(self.target_height)
                    
            else:
                self.get_logger().info("No aruco markers found")
                # Продолжаем регулирование даже без маркеров (используя предыдущие ошибки)
                self.ArucoRegulation(self.pitch_error, self.roll_error)
                # Если маркеры не видны более 10 секунд и дрон не на начальной высоте, поднимаемся выше
                if current_time - self.last_marker_seen > 10.0 and self.target_height * 1.2 < self.initial_target_height:
                    self.get_logger().info("No marker seen for 10 seconds, setting new target height")
                    # Увеличиваем целевую высоту на 20%
                    self.target_height *= 1.2
                    self.call_set_height_service(self.target_height)
                    self.last_marker_seen = current_time


            # Если целевая высота слишком низка (ниже 0.4 м) и дрон все еще в полете, начинаем посадку на платформу
            if self.target_height < 0.4 and self.takeoff:
                self.boarding = True
                # Отправляем нулевые скорости для остановки дрона
                twist = Twist()
                self.cmd_publisher.publish(twist)
                self.get_logger().info("Target height is too low, start boarding")
                # Вызов сервиса для посадки на платформу (board=True)
                self.call_takeoff_board_service(takeoff=False, board=True)
                self.takeoff = False


    def ArucoRegulation(self, pitch_error, roll_error):
        # Максимальное значение для PID-регуляторов
        max_pid = 1.0

        # Обновление и ограничение значения PID для тангажа
        self.pid_pitch.update_control(pitch_error)
        PID_PITCH = -self.pid_pitch.get_control()
        PID_PITCH = self.constrain(PID_PITCH, max_pid)

        # Обновление и ограничение значения PID для крена
        self.pid_roll.update_control(roll_error)
        PID_ROLL = self.pid_roll.get_control()  
        PID_ROLL = self.constrain(PID_ROLL, max_pid)

        print(f"PID Control: PITCH={PID_PITCH}, ROLL={PID_ROLL}")
        # Создание и публикация сообщения Twist для управления дроном
        cmd_msg = Twist()
        cmd_msg.linear.x = PID_PITCH  # Управление вперед/назад через тангаж
        cmd_msg.linear.y = PID_ROLL   # Управление влево/вправо через крен
        cmd_msg.angular.z = 0.0     # Отсутствие вращения
        self.cmd_publisher.publish(cmd_msg)

    def constrain(self, value, threshold):
        # Функция для ограничения значения в заданных пределах
        if value > threshold:
            value = threshold
        if value < -threshold:
            value = -threshold
        return value

    def call_set_height_service(self, target_height):
        # Ожидание доступности сервиса установки высоты
        while not self.takeoff_board_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for set_height service...")
        
        # Создание запроса на установку высоты
        request = SetHeight.Request()
        request.height = target_height
        # Установка флага снижения
        self.descending_flag = True

        # Асинхронный вызов сервиса и установка колбэка для обработки ответа
        future = self.set_height_client.call_async(request)
        future.add_done_callback(self.set_height_response_callback)

    def set_height_response_callback(self, future):
        # Обработка ответа от сервиса установки высоты
        try:
            response = future.result()
            if response.result:
                self.get_logger().info("Set height service call successful")
                # Сброс флага снижения после успешной установки высоты
                self.descending_flag = False
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def call_takeoff_board_service(self, takeoff: bool, board: bool):
        # Ждем, пока сервис станет доступным
        while not self.takeoff_board_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for take_off_board service...")
        # Создаём запрос на взлет или посадку
        request = TakeOffBoard.Request()
        request.takeoff = takeoff
        request.board = board

        # Отправляем запрос асинхронно и устанавливаем соответствующий колбэк
        future = self.takeoff_board_client.call_async(request)
        if takeoff:
            future.add_done_callback(self.takeoff_response_callback)
        else:
            future.add_done_callback(self.board_response_callback)
    
    def board_response_callback(self, future):
        # Обработка ответа от сервиса посадки на платформу
        try:
            response = future.result()
            if response.result:
                self.get_logger().info("Start Aruco Boarding")
                self.takeoff = False  # Флаг взлета сбрасывается после начала посадки
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def takeoff_response_callback(self, future):
        # Обработка ответа от сервиса взлета
        try:
            response = future.result()
            if response.result:
                self.get_logger().info('TakeOffBoard service call successful')
                time.sleep(8) # Ожидание стабилизации после взлета
                self.get_logger().info("Setting target height")
                self.call_set_height_service(self.initial_target_height) # Установка начальной целевой высоты
                time.sleep(4) # Дополнительное ожидание
                self.takeoff = True  # Установка флага взлета в True
            else:
                self.get_logger().warn('TakeOffBoard service call failed')
        except Exception as e:
            self.get_logger().error(f'Service call fasiled: {e}')


class PID:
    # Инициализация PID-регулятора
    def __init__(self, Kp, Ti, Td):
        self.Kp = Kp  # Пропорциональный коэффициент
        self.Td = Td  # Дифференциальный коэффициент
        self.Ti = Ti  # Интегральный коэффициент
        self.curr_error = 0 # Текущая ошибка
        self.prev_error = 0 # Предыдущая ошибка
        self.sum_error = 0  # Сумма ошибок (для интегральной составляющей)
        self.prev_error_deriv = 0 # Предыдущая производная ошибки
        self.curr_error_deriv = 0 # Текущая производная ошибки
        self.control = 0    # Выходное управляющее воздействие

    # Обновление управляющего воздействия PID-регулятора
    def update_control(self, current_error,
        reset_prev=False):
        self.prev_error = self.curr_error
        self.curr_error = current_error
        # Расчет интегральной ошибки
        self.sum_error = self.sum_error + self.curr_error
        # Расчет производной ошибки
        self.curr_error_deriv = self.curr_error - self.prev_error
        # Расчет управляющего воздействия PID
        self.control = self.Kp * self.curr_error + self.Ti * self.sum_error + self.Td * self.curr_error_deriv
    
    # Получение текущего управляющего воздействия
    def get_control(self):
        return self.control

def main(args=None):
    # Инициализация rclpy
    rclpy.init(args=args)
    # Создание экземпляра узла ArucoBoarding
    aruco_boarding = ArucoBoarding()
    # Запуск узла ROS 2
    rclpy.spin(aruco_boarding)
    # Остановка узла и завершение работы rclpy
    aruco_boarding.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()