from ara_api.ara_core import ARALinkManager
from ara_api.ara_vision import ARAVisionManager
from threading import Thread
import time

manager = ARALinkManager()  # создаём объект для управления квадрокоптером
vision = ARAVisionManager()  # создаём объект для работы с камерой

def read_aruco_data_in_flight():
    """
    Функция для чтения данных с камеры в полёте.
    """
    while True:
        aruco_data = vision.get_aruco_data()  # получаем данные о маркерах
        if aruco_data:
            print("Aruco data:", aruco_data)
        time.sleep(0.5)  # ждём 0.5 секунды перед следующим запросом

def main():
    aruco_thread = Thread(target=read_aruco_data_in_flight, daemon=True)
    aruco_thread.start()  # запускаем поток для камеры

    manager.takeoff(1.5)  # взлетаем на 1.5 метра
    time.sleep(5)  # ждём 5 секунд
    manager.set_velocity(1.5, 0)  # летим вперёд со скоростью 1.5 м/с
    time.sleep(2)  # движение 2 секунды
    manager.set_velocity(-0.5, 0)  # замедляемся
    time.sleep(2)  # пауза 2 секунды
    manager.set_velocity(0, -1.5)  # летим влево со скоростью 1.5 м/с
    time.sleep(4)  # движение 4 секунды
    manager.set_velocity(0, 0.5)  # замедляемся
    time.sleep(2)  # пауза 2 секунды
    manager.land()  # приземляемся

    print(vision.get_aruco_data())  # выводим финальные данные с камеры

if __name__ == "__main__":
    main()

