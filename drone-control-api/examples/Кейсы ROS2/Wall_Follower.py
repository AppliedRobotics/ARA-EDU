# Импорт необходимых библиотек ROS 2
import rclpy
from rclpy.node import Node
import time # Для задержек
from eagle_eye_msgs.srv import TakeOffBoard # Сервис для взлета/посадки
from geometry_msgs.msg import Twist # Сообщение для управления скоростью
from std_msgs.msg import Float32MultiArray # Сообщение для данных с ультразвуковых датчиков

class WallFollower(Node):
    def __init__(self):
        super().__init__("wall_follower")
        self.get_logger().info("Wall Follower node initialized")
        # Издатель для отправки команд скорости
        self.cmd_publisher = self.create_publisher(Twist, "cmd_vel", 10)
        # Клиент сервиса для взлета/посадки
        self.takeoff_board_client = self.create_client(TakeOffBoard, "take_off_board")
        # Подписчик на данные с ультразвуковых датчиков
        self.ultrasonic_subscriber = self.create_subscription(Float32MultiArray, "ultrasonic", self.distance_callback, 10)

        # Желаемое расстояние до стены
        self.desired_distance = 0.5
        # Переменная для хранения ошибки расстояния
        self.distance_error = 0.0

        # Инициализация PID-регулятора для удержания на нужном расстоянии
        self.pid_pitch = PID(2.5, 0.0005, 0.5)
    
        # Скорость движения вдоль стены (используется как константа для бокового движения)
        self.roll_speed = 0.8

        # Вызов сервиса взлета в начале работы узла
        self.call_takeoff_board_service()

        # Флаг, указывающий на завершение взлета
        self.takeoff = False

        # Сообщение Twist с нулевыми скоростями для остановки дрона
        self.zero_twist = Twist()

    def distance_callback(self, msg: Float32MultiArray):
        # Обработка данных с ультразвукового датчика только после взлета
        if self.takeoff:
            # Получаем расстояние до стены (первый элемент массива, конвертируем из см в метры)
            distance_to_wall = msg.data[0]/100
            # Вычисляем ошибку расстояния (текущее - желаемое)
            self.distance_error = distance_to_wall - self.desired_distance 
            # Вызов функции регулирования движения вдоль стены
            self.WallRegulation(self.distance_error)
        else:
            self.get_logger().info("Takeoff not completed")
            # Если взлет не завершен, отправляем нулевые скорости
            self.cmd_publisher.publish(self.zero_twist)

    def constrain(self, value, threshold):
        # Функция для ограничения значения в заданных пределах
        if value > threshold:
            value = threshold
        if value < -threshold:
            value = -threshold
        return value

    def WallRegulation(self, pitch_error):
        # Точность регулирования (порог, в пределах которого считается, что дрон находится на нужном расстоянии)
        accuracy = 0.1
        # Максимальное значение управляющего воздействия PID
        max_pid = 1.0

        # Если ошибка расстояния превышает порог точности
        if abs(pitch_error) > accuracy:
            self.pid_pitch.update_control(pitch_error) # Обновляем PID-регулятор
            PID_PITCH = self.pid_pitch.get_control() # Получаем управляющее воздействие
            PID_PITCH = self.constrain(PID_PITCH, max_pid) # Ограничиваем управляющее воздействие
            ROLL = 0.0 
        else:
            # Если дрон находится в пределах точности, останавливаем движение вперед/назад и начинаем боковое движение
            PID_PITCH = 0.0 # Отсутствие движения вперед/назад
            ROLL = self.roll_speed # Боковое движение (влево/вправо) для следования стене

        print(f"PID_PITCH: {PID_PITCH}, ROLL: {ROLL}")

        # Создаем и публикуем сообщение Twist для управления дроном
        cmd_msg = Twist()
        cmd_msg.linear.x = PID_PITCH # Линейная скорость по оси X (вперед/назад)
        cmd_msg.linear.y = ROLL # Линейная скорость по оси Y (влево/вправо)
        cmd_msg.angular.z = 0.0 # Угловая скорость по оси Z (вращение)
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
    """
    Класс PID-регулятора для управления системой с обратной связью
    Реализует пропорционально-интегрально-дифференциальное управление
    """
    
    def __init__(self, Kp, Ti, Td):
        """
        Инициализация PID-регулятора
        
        Args:
            Kp (float): Пропорциональный коэффициент
            Ti (float): Интегральный коэффициент
            Td (float): Дифференциальный коэффициент
        """
        # Коэффициенты PID-регулятора
        self.Kp = Kp  # Пропорциональный коэффициент
        self.Td = Td  # Дифференциальный коэффициент
        self.Ti = Ti  # Интегральный коэффициент
        
        # Переменные для хранения ошибок
        self.curr_error = 0          # Текущая ошибка
        self.prev_error = 0          # Предыдущая ошибка
        self.sum_error = 0           # Накопленная сумма ошибок (для интегральной составляющей)
        self.prev_error_deriv = 0    # Предыдущая производная ошибки
        self.curr_error_deriv = 0    # Текущая производная ошибки
        self.control = 0             # Выходной сигнал регулятора
    
    def update_control(self, current_error, reset_prev=False):
        """
        Обновляет выходной сигнал PID-регулятора на основе текущей ошибки
        
        Args:
            current_error (float): Текущее значение ошибки
            reset_prev (bool): Флаг сброса предыдущих значений (не используется)
        """
        # Сохраняем предыдущую ошибку и обновляем текущую
        self.prev_error = self.curr_error
        self.curr_error = current_error
        
        # Вычисляем интегральную составляющую (накопление ошибки)
        self.sum_error = self.sum_error + self.curr_error
        
        # Вычисляем дифференциальную составляющую (скорость изменения ошибки)
        self.curr_error_deriv = self.curr_error - self.prev_error
        
        # Вычисляем итоговый управляющий сигнал PID
        # U(t) = Kp*e(t) + Ki*∫e(t)dt + Kd*de(t)/dt
        self.control = (self.Kp * self.curr_error + 
                       self.Ti * self.sum_error + 
                       self.Td * self.curr_error_deriv)
    
    def get_control(self):
        """
        Возвращает текущее значение управляющего сигнала
        
        Returns:
            float: Значение управляющего сигнала
        """
        return self.control


def main(args=None):
    # Инициализация rclpy
    rclpy.init(args=args)
    # Создание экземпляра узла WallFollower
    wall_follower = WallFollower()
    try:
        # Запуск узла ROS 2
        rclpy.spin(wall_follower)
    except KeyboardInterrupt:
        pass
    finally:
        # Корректное завершение работы узла и rclpy
        wall_follower.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
