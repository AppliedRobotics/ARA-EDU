#!/usr/bin/env python3
"""
Узел для следования за объектом (блобом) с использованием компьютерного зрения
Этот модуль реализует ROS2 узел, который управляет дроном для следования за цветными объектами
"""

import rclpy
from rclpy.node import Node
from eagle_eye_msgs.msg import Target, BoundingBox
from eagle_eye_msgs.srv import TakeOffBoard
from geometry_msgs.msg import Twist
import time

# Команда для настройки автоэкспозиции камеры
# v4l2-ctl -c ae_low_limit=30,ae_high_limit=40


class BlobFollower(Node):
    """
    Класс для управления дроном с целью следования за цветными объектами (блобами)
    Использует PID-регуляторы для стабилизации движения и компьютерное зрение для обнаружения целей
    """
    def __init__(self):
        """
        Инициализация узла BlobFollower
        Настраивает издателей, подписчиков, клиентов сервисов и PID-регуляторы
        """
        super().__init__("blob_follower")
        self.get_logger().info("Fixed Blob Follower node initialized")
        
        # Создаем издателя для команд управления
        self.cmd_publisher = self.create_publisher(Twist, "cmd_vel", 10)
        
        # Создаем клиента для сервиса взлета/посадки
        self.takeoff_board_client = self.create_client(TakeOffBoard, "/take_off_board")
        
        # Подписываемся на топик с обнаруженными объектами
        self.target_subscriber = self.create_subscription(Target, "/blobs", self.target_callback, 10)

        # PID-регуляторы
        self.pid_pitch = PID(0.003, 0.00001, 0.001)  
        self.pid_roll = PID(0.005, 0.0001, 0.005)   

        # Желаемая площадь объекта (в пикселях) для поддержания расстояния
        self.target_area = 1000

        # Мертвая зона 
        self.dead_zone_area = 50    # ±50 пикселей
        self.dead_zone_center = 20  # ±20 пикселей по центру

        # Центр изображения камеры
        self.image_center_x = 160
        self.image_center_y = 120

        # Флаг готовности к полету
        self.takeoff = False
       
       # Переменная для хранения текущей цели
        self.target = Target()

        self.last_area = 0
        self.last_center_x = 0

        # Запускаем процедуру взлета
        self.call_takeoff_board_service()

    def target_callback(self, msg: Target):
        """
        Callback-функция для обработки сообщений с обнаруженными объектами
        
        Args:
            msg (Target): Сообщение содержащее информацию об обнаруженных объектах
        """
        if self.takeoff:
             # Дрон готов к полету, обрабатываем обнаруженные объекты
            self.target = msg
            boxes = self.target.boxes
            
            if len(boxes) > 0:
                box = boxes[0]
                
                self.get_logger().info(f"Box area: {box.area}, Center: ({box.center.x}, {box.center.y})")
                
                # Вычисляем ошибки для PID-регуляторов
                # Ошибка по горизонтали: разность между центром объекта и центром изображения
                self.roll_error = box.center.x - self.image_center_x
                # Ошибка по расстоянию: разность между желаемой и текущей площадью объекта
                self.pitch_error = self.target_area - box.area
                # Если площадь больше целевой - объект близко - нужно отлететь назад
                # Если площадь меньще целевой - объект далеко - нужно подлететь вперед
                self.last_area = box.area
                self.last_center_x = box.center.x
                
                # Применяем регулирование для следования за объектом
                self.BlobRegulation(self.pitch_error, self.roll_error)
            else:
                self.get_logger().warn("No objects detected")
                # Если объектов нет - останавливаемся
                self.stop_movement()
        else:
            self.get_logger().info("Waiting for takeoff completion...")

    def stop_movement(self):
  
        cmd_msg = Twist()
        self.cmd_publisher.publish(cmd_msg)

    def constrain(self, value, threshold):
 
        if value > threshold:
            return threshold
        if value < -threshold:
            return -threshold
        return value

    def apply_dead_zone(self, error, dead_zone):
        if abs(error) < dead_zone:
            return 0.0
        return error

    def BlobRegulation(self, pitch_error, roll_error):
        """
        Основная функция регулирования движения дрона для следования за объектом
        Использует PID-регуляторы для вычисления команд управления
        
        Args:
            pitch_error (float): Ошибка по расстоянию (разность площадей)
            roll_error (float): Ошибка по горизонтальному положению
        """
        
        # Применяем мертвые зоны
        pitch_error_clean = self.apply_dead_zone(pitch_error, self.dead_zone_area)
        roll_error_clean = self.apply_dead_zone(roll_error, self.dead_zone_center)
        
        # Максимальные значения PID
        max_pid_linear = 0.5   
        max_pid_angular = 0.3

        # Регулирование по расстоянию 
        if abs(pitch_error_clean) > 0:
            self.pid_pitch.update_control(pitch_error_clean)
            PID_PITCH = self.pid_pitch.get_control()
            PID_PITCH = self.constrain(PID_PITCH, max_pid_linear)
        else:
            PID_PITCH = 0.0

        # Регулирование по горизонтали 
        if abs(roll_error_clean) > 0:
            self.pid_roll.update_control(roll_error_clean)
            PID_ROLL = self.pid_roll.get_control()  
            PID_ROLL = self.constrain(PID_ROLL, max_pid_linear)
        else:
            PID_ROLL = 0.0

        # Отладочная информация
        self.get_logger().info(
            f"Errors - Area: {self.last_area}, Pitch_err: {pitch_error:.1f}, "
            f"Roll_err: {roll_error:.1f} | Controls - Pitch: {PID_PITCH:.3f}, Roll: {PID_ROLL:.3f}"
        )
        
        # Формируем команду для управления движением
        cmd_msg = Twist()
        cmd_msg.linear.x = PID_PITCH   
        cmd_msg.linear.y = PID_ROLL    
        cmd_msg.angular.z = 0.0
         # Отправляем команду управления
        self.cmd_publisher.publish(cmd_msg)

    def call_takeoff_board_service(self):
        """
        Инициирует процедуру взлета дрона через сервис
        Отправляет асинхронный запрос на взлет и настраивает callback для обработки ответа
        """
         # Ждем, пока сервис взлета/посадки станет доступным
        while not self.takeoff_board_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for take_off_board service...")
         # Создаём запрос на взлет
        request = TakeOffBoard.Request()
        request.takeoff = True  # Запрашиваем взлет
        request.board = False   # Не запрашиваем посадку

        # Отправляем запрос асинхронно (не блокируем выполнение)
        future = self.takeoff_board_client.call_async(request)
        # Устанавливаем callback-функцию для обработки ответа сервиса
        future.add_done_callback(self.takeoff_board_response_callback)
    
    def takeoff_board_response_callback(self, future):
        """
        Callback-функция для обработки ответа сервиса взлета/посадки
        
        Args:
            future: Объект Future содержащий результат вызова сервиса
        """
        try:
            response = future.result()
            if response.result:
                self.get_logger().info('Takeoff successful')
                # Ждем 10 секунд для стабилизации дрона после взлета
                time.sleep(10)  
                # Устанавливаем флаг готовности к полету
                self.takeoff = True
                self.stop_movement()
            else:
                self.get_logger().error('Takeoff failed')
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
        self.Kp = Kp
        self.Td = Td
        self.Ti = Ti
        # Переменные для хранения ошибок
        self.curr_error = 0          # Текущая ошибка
        self.prev_error = 0          # Предыдущая ошибка
        self.sum_error = 0           # Накопленная сумма ошибок (для интегральной составляющей)
        self.prev_error_deriv = 0    # Предыдущая производная ошибки
        self.curr_error_deriv = 0    # Текущая производная ошибки
        self.control = 0             # Выходной сигнал регулятора
    
        # Ограничение интегральной составляющей
        self.max_integral = 1000
    
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
        
         # Вычисляем интегральную составляющую (накопление ошибки) с ограничением 
        self.sum_error += self.curr_error
        if abs(self.sum_error) > self.max_integral:
            self.sum_error = self.max_integral if self.sum_error > 0 else -self.max_integral
        # Вычисляем дифференциальную составляющую (скорость изменения ошибки)
        self.curr_error_deriv = self.curr_error - self.prev_error
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
    """
    Главная функция для запуска узла BlobFollower
    
    Args:
        args: Аргументы командной строки (по умолчанию None)
    """
     # Инициализация ROS2
    rclpy.init(args=args)
    # Создание экземпляра узла
    node = BlobFollower()
    
    try:
        # Запуск основного цикла обработки сообщений
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by user")
    finally:
        node.stop_movement()
        # Корректное завершение работы
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()