import rclpy
from rclpy.node import Node
from eagle_eye_msgs.srv import TakeOffBoard
from geometry_msgs.msg import Twist
import time
import math


class SimpleDroneControl(Node):
    def __init__(self):
        super().__init__("simple_drone_control")
        self.get_logger().info("Simple Drone Control node started")
       
        # Создаем клиент для сервиса взлета/посадки
        self.takeoff_client = self.create_client(TakeOffBoard, "/take_off_board")
       
        # Создаем издатель для управления скоростью
        self.cmd_publisher = self.create_publisher(Twist, "cmd_vel", 10)
       
        # Запускаем последовательность
        self.execute_sequence()


    def execute_sequence(self):
        """Основная последовательность управления дроном"""
       
        #  Установка соединения и взлет
        self.call_takeoff_service(takeoff=True, board=False)
        time.sleep(8)  # Ждем завершения взлета
       
        #  Задержка 10 секунд
        time.sleep(10)
       
        # Поворот на 1.57 радиан (90 градусов) по часовой стрелке
        self.rotate_drone(1.57)  # 1.57 радиан = 90 градусов
       
        #  Разрыв соединения (посадка)
        self.call_takeoff_service(takeoff=False, board=True)
       



    def call_takeoff_service(self, takeoff: bool, board: bool):
        """Вызов сервиса взлета/посадки"""
        # Ждем доступности сервиса
        while not self.takeoff_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for take_off_board service...")
       
        # Создаем запрос
        request = TakeOffBoard.Request()
        request.takeoff = takeoff
        request.board = board
       
        # Отправляем запрос и ждем ответ
        future = self.takeoff_client.call_async(request)
        while not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
       
        try:
            response = future.result()
            if response.result:
                if takeoff:
                    self.get_logger().info("Takeoff successful")
                else:
                    self.get_logger().info("Landing successful")
            else:
                self.get_logger().error("Service call failed")
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')


    def rotate_drone(self, angle_rad):
        """Поворот дрона на заданный угол в радианах"""
        # Создаем сообщение для поворота
        twist_msg = Twist()
       
        # Устанавливаем угловую скорость по Z (рыскание)
        # Отрицательное значение - поворот по часовой стрелке
        angular_speed = -0.5  # рад/сек
        twist_msg.angular.z = angular_speed
       
        # Рассчитываем время для поворота на нужный угол
        rotation_time = abs(angle_rad / angular_speed)
       
        # Публикуем команду поворота
        self.get_logger().info(f"Rotating for {rotation_time:.2f} seconds")
        start_time = time.time()
       
        while time.time() - start_time < rotation_time:
            self.cmd_publisher.publish(twist_msg)
            time.sleep(0.1)
       
        # Останавливаем вращение
        stop_msg = Twist()
        stop_msg.angular.z = 0.0
        self.cmd_publisher.publish(stop_msg)
        self.get_logger().info("Rotation completed")


def main(args=None):
    rclpy.init(args=args)
    node = SimpleDroneControl()
   
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()