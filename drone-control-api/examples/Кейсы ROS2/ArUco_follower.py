# Импорт необходимых библиотек ROS 2
import rclpy
from rclpy.node import Node
from eagle_eye_msgs.msg import MarkerArray # Сообщение для массива маркеров ArUco
from eagle_eye_msgs.srv import TakeOffBoard # Сервис для взлета/посадки
from geometry_msgs.msg import Twist # Сообщение для управления скоростью
import time # Для задержек

class ArucoFollower(Node):
    def __init__(self):
        super().__init__("aruco_follower")
        self.get_logger().info("Aruco Follower node initialized")
        # Подписка на топик с маркерами ArUco
        self.marker_sub = self.create_subscription(MarkerArray, "arucos", self.marker_callback, 10)
        # Издатель для отправки команд скорости
        self.cmd_publisher = self.create_publisher(Twist, "cmd_vel", 10)
        # Создаем клиент сервиса для взлета/посадки
        self.takeoff_board_client = self.create_client(TakeOffBoard, "/take_off_board")
        # Создаем переменную для хранения желаемого расстояния до маркера
        self.desired_distance = 1.1
        # Инициализируем переменные для ошибок регулирования по осям X (крен), Z (расстояние) и вращению (рыскание)
        self.roll_error = 0.0
        self.pitch_error = 0.0
        self.yaw_error = 0.0

        # Инициализация PID-регуляторов для крена, тангажа (движение вперед/назад) и рыскания
        self.pid_roll = PID(1.0, 0.002, 0.2)
        self.pid_pitch = PID(1.0, 0.002, 0.2)
        self.pid_yaw = PID(0.6, 0.00, 0.5)

        # Флаг состояния взлета
        self.takeoff = False

        # Вызов сервиса для взлета
        self.call_takeoff_board_service()

        # Инициализируем и публикуем нулевые скорости для остановки дрона
        test_msg = Twist()
        test_msg.linear.x = 0.0
        test_msg.linear.y = 0.0
        test_msg.angular.z = 0.0
        self.cmd_publisher.publish(test_msg)

    def marker_callback(self, msg):
        # Обработка маркеров только после успешного взлета
        if self.takeoff:
            aruco_markers = msg.markers
            # Если обнаружены маркеры
            if len(aruco_markers) > 0:
                # Получаем ошибку по оси X (крен) из первого маркера
                self.roll_error = aruco_markers[0].pose.pose.position.x
                # Получаем расстояние до маркера по оси Z
                distance_to_marker = aruco_markers[0].pose.pose.position.z
                # Вычисляем ошибку по тангажу (разница между текущим и желаемым расстоянием)
                self.pitch_error = distance_to_marker - self.desired_distance
                # Получаем ошибку по рысканию (ориентация)
                self.yaw_error = aruco_markers[0].pose.pose.orientation.z
                self.get_logger().info(f"Roll error: {self.roll_error}, Pitch error: {self.pitch_error}")
                # Вызов функции регулирования на основе ошибок
                self.ArucoRegulation(self.pitch_error, self.roll_error, self.yaw_error)
            else:
                self.get_logger().info("No aruco markers found")
                # Если маркеры не найдены, продолжаем регулирование с последними известными ошибками
                self.ArucoRegulation(self.pitch_error, self.roll_error, self.yaw_error)

    def constrain(self, value, threshold):
        # Функция для ограничения значения в заданных пределах
        if value > threshold:
            value = threshold
        if value < -threshold:
            value = -threshold
        return value
    

    def ArucoRegulation(self, pitch_error, roll_error, yaw_error):
        # Порог точности для ошибок регулирования
        accuracy = 0.1
        # Максимальное значение управляющего воздействия PID
        max_pid = 1.0

        # Регулирование по тангажу (движение вперед/назад)
        if abs(pitch_error) > accuracy:
            self.pid_pitch.update_control(pitch_error)
            PID_PITCH = self.pid_pitch.get_control()
            PID_PITCH = self.constrain(PID_PITCH, max_pid)
        else:
            PID_PITCH = 0.0

        # Регулирование по крену (движение влево/вправо)
        if abs(roll_error) > accuracy:
            self.pid_roll.update_control(roll_error)
            PID_ROLL = self.pid_roll.get_control()  
            PID_ROLL = self.constrain(PID_ROLL, max_pid)
        else:
            PID_ROLL = 0.0

        # Регулирование по рысканию (вращение)
        if abs(yaw_error) > 0.1:
            self.pid_yaw.update_control(yaw_error)
            PID_YAW = - self.pid_yaw.get_control()
            PID_YAW = self.constrain(PID_YAW, 0.3)

        else:
            PID_YAW = 0.0

        print(f"PID Control: PITCH={PID_PITCH}, ROLL={PID_ROLL}, YAW = {PID_YAW}")
        # Создаем и публикуем сообщение Twist для управления дроном
        cmd_msg = Twist()
        cmd_msg.linear.x = PID_PITCH # Линейная скорость по оси X (вперед/назад)
        cmd_msg.linear.y = PID_ROLL # Линейная скорость по оси Y (влево/вправо)
        cmd_msg.angular.z = PID_YAW # Угловая скорость по оси Z (вращение)
        self.cmd_publisher.publish(cmd_msg)


    def call_takeoff_board_service(self):
        # Ожидание доступности сервиса взлета/посадки
        while not self.takeoff_board_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for take_off_board service...")
        # Создаём запрос на взлет
        request = TakeOffBoard.Request()
        request.takeoff = True 
        request.board = False 

        # Отправляем запрос асинхронно
        future = self.takeoff_board_client.call_async(request)

        # Устанавливаем колбэк для обработки ответа сервиса
        future.add_done_callback(self.takeoff_board_response_callback)
    
    def takeoff_board_response_callback(self, future):
        # Обработка ответа от сервиса взлета/посадки
        try:
            response = future.result()
            if response.result:
                self.get_logger().info('TakeOffBoard service call successful')
                time.sleep(10) # Задержка для стабилизации дрона после взлета
                self.takeoff = True # Установка флага взлета в True
            else:
                self.get_logger().warn('TakeOffBoard service call failed')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')



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
    # Создание экземпляра узла ArucoFollower
    node = ArucoFollower()
    # Запуск узла ROS 2
    rclpy.spin(node)
    # Корректное завершение работы узла и rclpy
    rclpy.shutdown()

if __name__ == '__main__':
    main()
