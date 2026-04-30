import rclpy
from rclpy.node import Node
from eagle_eye_msgs.srv import TakeOffBoard # Сервис для взлета/посадки
import time # Для задержек


class SimpleTakeOffLand(Node):
    def __init__(self):
        super().__init__("simple_takeoff_land")
        self.get_logger().info("Simple TakeOff and Land node started")
       
        # Создаем клиент для сервиса взлета/посадки
        self.takeoff_client = self.create_client(TakeOffBoard, "/take_off_board")
       
        self.execute_sequence()


    def execute_sequence(self):
       
        # Ждем ответа сервиса
        self.get_logger().info("Waiting for service...")
        while not self.takeoff_client.wait_for_service(timeout_sec=1.0):
            pass
       
        # Взлет
        self.get_logger().info("Taking off...")
        request = TakeOffBoard.Request()
        request.takeoff = True
        request.board = False
        future = self.takeoff_client.call_async(request)
       
        # Ожидание завершения взлета
        while not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        time.sleep(10)
       
        self.get_logger().info("Landing...")
        request = TakeOffBoard.Request()
        request.takeoff = False
        request.board = True
        future = self.takeoff_client.call_async(request)
       
        # Ожидание завершения посадки
        while not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
       



def main(args=None):
    rclpy.init(args=args)
    node = SimpleTakeOffLand()
   
    try:
        rclpy.spin(node)
    except:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()